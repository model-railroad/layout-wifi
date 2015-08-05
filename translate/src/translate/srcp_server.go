package translate

import (
    "bufio"
    "flag"
    "fmt"
    "net"
    "strconv"
    "strings"
    "sync"
    "time"
)

var SRCP_PORT = flag.String("srcp-port", ":4303", "SRCP server host:port")

const SRCP_MODE_HANDSHAKE = "HANDSHAKE"
const SRCP_MODE_INFO      = "INFO"
const SRCP_MODE_COMMAND   = "COMMAND"
const SRCP_HEADER         = "SRCP 0.8.4; SRCPOTHER 0.8.3"

const SRCP_BUS_TURNOUTS   = 7 // bus used by SRCP Client to control the turnouts
const SRCP_BUS_SENSORS    = 8 // bus used by SRCP Client to poll sensors

//-----

type SrcpSession struct {
    id       int
    mode     string
    time     int
    conn     net.Conn
    sensors  []uint16
    sessions *SrcpSessions
}

func NewSrcpSession(id int, mode string, time int, conn net.Conn, sessions *SrcpSessions) *SrcpSession {
    s := &SrcpSession{id, mode, time, conn, make([]uint16, MAX_AIUS), sessions}
    return s
}

func (s *SrcpSession) ReplyRaw(str string) {
    str = fmt.Sprintf("%s\n", str)
    fmt.Printf("[SRCP %d] < %s", s.id, str)
    s.conn.Write( []byte(str) )
}

func (s *SrcpSession) Reply(str string) {
    s.time += 1
    s.ReplyRaw(fmt.Sprintf("%d %s", s.time, str))
}

//-----

type SrcpSessions struct {
    mutex    sync.Mutex
    sessions map[int] *SrcpSession
}

func NewSrcpSessions() *SrcpSessions {
    s := &SrcpSessions{}
    s.sessions = make(map[int] *SrcpSession)
    return s
}

func (s *SrcpSessions) Add(session *SrcpSession) {
    s.mutex.Lock()
    defer s.mutex.Unlock()

    s.sessions[session.id] = session
}

func (s *SrcpSessions) Remove(session *SrcpSession) {
    s.mutex.Lock()
    defer s.mutex.Unlock()

    delete(s.sessions, session.id)
}

func (s *SrcpSessions) Iter(f func(*SrcpSession)) {
    s.mutex.Lock()
    defer s.mutex.Unlock()

    for _, v := range s.sessions {
        if v != nil {
            f(v)
        }
    }
}

func (s *SrcpSessions) String() string {
    str := ""

    s.mutex.Lock()
    defer s.mutex.Unlock()

    for k, v := range s.sessions {
        if v != nil {
            str += strconv.Itoa(k) + " "
        }
    }
    
    return strings.TrimSpace(str)
}

//-----

func SrcpServer(m *Model) {
    fmt.Println("Start SRCP server")

    listener, err := net.Listen("tcp", *SRCP_PORT)
    if err != nil {
        panic(err)
    }
    
    go func(listener net.Listener) {
        defer listener.Close()

        sessions := NewSrcpSessions()
        session_id := 0
        
        for !m.IsQuitting() {
            conn, err := listener.Accept()
            if err != nil {
                panic(err)
            }
            session_id += 1
            go func(id int, conn net.Conn) {
                session := NewSrcpSession(id, SRCP_MODE_HANDSHAKE, 0, conn, sessions)
                sessions.Add(session)
                defer sessions.Remove(session)
                HandleSrcpConn(m, session)
            }(session_id, conn)
        }
    }(listener)
}

func HandleSrcpConn(m *Model, s *SrcpSession) {
    fmt.Println("[SRCP] New session/connection")

    defer s.conn.Close()

    s.ReplyRaw(SRCP_HEADER)
    
    r := bufio.NewReader(s.conn)

    loopRead: for !m.IsQuitting() {
        if s.mode == SRCP_MODE_INFO {
            s.conn.SetReadDeadline(time.Now().Add(500 * time.Millisecond))
        }
        line, err := r.ReadString('\n')

        if s.mode == SRCP_MODE_INFO && err != nil {
            if e, ok := err.(*net.OpError); ok && e.Timeout() {
                // send periodic INFO sensor updates
                SendSrcpSensorUpdate(m, s)
                continue loopRead
            }
        }

        if err == nil {
            line := strings.TrimSpace(line)
            fmt.Printf("[SRCP %d] > %s\n", s.id, line)
            err = HandleSrcpLine(m, s, line)
        }

        if err != nil {
            if op, ok := err.(*net.OpError); ok {
                fmt.Println("[SRCP] Connection error:", op.Op)
                break loopRead
            } else {
                panic(err)
            }
        }
    }
    
    fmt.Println("[SRCP] Connection closed")
}

func HandleSrcpLine(m *Model, s *SrcpSession, line string) error {

    if strings.Index(line, "SET PROTOCOL SRCP ") == 0 {
        s.Reply("201 OK PROTOCOL SRCP")
        
    } else if strings.HasPrefix(line, "SET CONNECTIONMODE SRCP ") {
        s.mode = line[ len("SET CONNECTIONMODE SRCP ") : ]
        s.Reply("202 OK CONNECTIONMODE")
                
    } else if line == "GO" {
        s.Reply("200 OK GO " + strconv.Itoa(s.id))
        if s.mode == SRCP_MODE_INFO {
            s.Reply("100 INFO 0 DESCRIPTION SESSION SERVER TIME")

            // time ratio fx:fy
            s.Reply("100 INFO 0 TIME 1 1")
            // time julianDay HH MM SS... just mock it
            s.Reply("100 INFO 0 TIME 12345 23 59 59")

            // power is always on and not really controllable
            s.Reply("100 INFO 1 POWER ON controlled by digix")

            // bus 7 is SRCP_BUS_TURNOUTS
            s.Reply("100 INFO 7 DESCRIPTION GA DESCRIPTION")
            // give the initial turnout states
            states := m.GetTurnoutStates()
            for t := 1; t <= MAX_TURNOUTS; t++ {
                normal := (states & 1)
                states >>= 1
                if normal == 1 {
                    // TODO only send reversed turnouts. This may not be up to spec.
                    //
                    // turnout N port 0 for normal, value 1 if normal
                    // turnout N port 1 for reverse, value 1 if reverse
                    s.Reply(fmt.Sprintf("100 INFO 7 GA %d 0 %d", t, 1 - normal))
                    s.Reply(fmt.Sprintf("100 INFO 7 GA %d 1 %d", t, normal))
                }
            }

            // bus 8 is SRCP_BUS_SENSORS
            s.Reply("100 INFO 8 DESCRIPTION FB DESCRIPTION")
            // give the initial sensors states *if they are not zero*
            // (c.f. SRCP-084.PDF pages 33-34: don't transmit default state.)
            for t := 1; t <= MAX_SENSORS; t++ {               
                if m.GetSensor(t) {
                    s.Reply(fmt.Sprintf("100 INFO 8 FB %d 1", t))
                }
            }
        }
    } else if strings.HasPrefix(line, "SET 7 GA ") {
        line = line[ len("SET 7 GA ") : ]
        fields := strings.Fields(line)
        if len(fields) >= 2 {
            turnout, err := strconv.Atoi(fields[0])
            if err == nil && turnout >= 1 && turnout <= MAX_TURNOUTS {
                value, err := strconv.Atoi(fields[1])
                if err == nil && value >= 0 && value <= 1 {
                    // Toggle turnout
                    m.SendTurnoutOp( &TurnoutOp{turnout, value == 0} )
                }
            }
        }
        s.Reply("200 OK")
    } else {
        // Ignore unknown commands
        s.Reply("200 OK")
    }

    return nil
}

func SendSrcpSensorUpdate(m *Model, s *SrcpSession) {
    // send periodic INFO sensor updates if they have changed
    sensor := 0
    for aiu := 1; aiu <= MAX_AIUS; aiu++ {
        state := m.GetSensors(aiu)
        previous := s.sensors[aiu - 1]
        if state != previous {
            s.sensors[aiu - 1] = state
            for i := 1; i <= SENSORS_PER_AIU; i++ {
                t := state & 1
                if t != (previous & 1) {
                    s.Reply(fmt.Sprintf("100 INFO 8 FB %d %d", sensor + i, t))
                }
                state    >>= 1
                previous >>= 1
            }
        }
        sensor += SENSORS_PER_AIU
    }
}