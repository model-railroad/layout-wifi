package translate 

/*
    Simulates the LayoutWifi server running on a DigiX.
    Note that the DigiX USR-WIFI232 module has a peculiar mode of
    operation: all input from incoming connections are serialized
    onto the serial port, and outputs to the serial ports are sent
    to all current connections in parallel.
*/

import (
    "bufio"
    "flag"
    "fmt"
    "io"
    "net"
    "strings"
    "sync"
    "time"
)

var LW_SERV_PORT = flag.String("lw-serv-port", ":9090", "LayoutWifi simulator server host:port")

const LW_LED_PIN        = 13
const LW_RELAY_PIN_1    = 90
const LW_TURNOUT_N      = 10
const LW_RELAY_N        =  2 * LW_TURNOUT_N

const LW_SENSORS_PER_AIU = SENSORS_PER_AIU

const LW_SENSOR_PIN_1   = 107
const LW_SENSOR_N       =  45
const LW_AIU_N          =   4

const LW_TURNOUT_NORMAL    = 'N'
const LW_TURNOUT_REVERSE   = 'R'

func LW_RELAY_NORMAL(T uint) uint {
    return 2 * (T)
}

func LW_RELAY_REVERSE(T uint) uint {
    return ((2 * (T)) + 1)
}

func LW_READ_BIT( I  uint16, N uint) uint16 {
    return (((I) &  (1<<N)) >> N)
}
func LW_SET_BIT(  I  uint16, N uint) uint16 {
    return ((I) |  (1<<N))
}
func LW_CLEAR_BIT(I  uint16, N uint) uint16 {
    return ((I) & ^(1<<N))
}

//-----

type LwSensor struct {
    Sensor  uint
    Active  bool
}

type LwServ struct {
    _sensors    []uint16
    _last       []uint16
    sensors_chan chan LwSensor
    reader_chan  chan string
    writer_chan  chan string

    mutex       sync.Mutex
    writer_id   int
    writers     map[int]chan string
}

func NewLwServ() *LwServ {
    lw := &LwServ{}
    lw._sensors = make([]uint16, LW_AIU_N)
    lw._last    = make([]uint16, LW_AIU_N)
    lw.sensors_chan = make(chan LwSensor, 32)
    lw.reader_chan  = make(chan string, 32)
    lw.writer_chan  = make(chan string, 32)
    lw.writers      = make(map[int]chan string)
    
    for i := 0; i < LW_AIU_N; i++ {
        lw._last[i] = uint16(0xFFFF)
    }

    return lw
}

func (s *LwServ) AllocWriterChan() (id int, c chan string) {
    c = make(chan string, 32)
    s.mutex.Lock()
    defer s.mutex.Unlock()
    s.writer_id += 1
    s.writers[s.writer_id] = c
    return s.writer_id, c
}

func (s *LwServ) ReleaseWriterChan(id int) {
    s.mutex.Lock()
    defer s.mutex.Unlock()
    delete(s.writers, id)
}

func (s *LwServ) SendWriters(line string) {
    s.mutex.Lock()
    defer s.mutex.Unlock()

    for _, w := range s.writers {
        w <- line
    }
}

//-----

func LwServer(m *Model) chan LwSensor {
    fmt.Println("Start LW server")

    listener, err := net.Listen("tcp", *LW_SERV_PORT)
    if err != nil {
        panic(err)
    }
    
    serv := NewLwServ()

    go LwServHandler(m, serv)
    
    go func(listener net.Listener) {
        defer listener.Close()

        for !m.IsQuitting() {
            conn, err := listener.Accept()
            if err != nil {
                panic(err)
            }
            
            go func(conn net.Conn) {
                id, c := serv.AllocWriterChan()
                defer serv.ReleaseWriterChan(id)
                HandleLwServConn(conn, m, serv, id, c)
            }(conn)
        }
    }(listener)
    
    return serv.sensors_chan
}

func LwServHandler(m *Model, s *LwServ) {
    fmt.Println("[LW-SERV] New handler")

    for !m.IsQuitting() {
        select {
        case line := <- s.reader_chan:
            fmt.Printf("[LW-SERV] < %s\n", line)
            s.writer_chan <- HandleLwServLine(s, line)

        case <- time.After(500 * time.Millisecond):
            s.writer_chan <- LwServPollSensors(m, s)

        case line := <- s.writer_chan:
            s.SendWriters(line)
        }
    }

    fmt.Println("[LW-SERV] End handler")
}

func HandleLwServConn(conn net.Conn, m *Model, s *LwServ, id int, writer chan string) {
    fmt.Println("[LW-SERV] New connection")

    defer conn.Close()

    r := bufio.NewReader(conn)

    loopRead: for !m.IsQuitting() {
        conn.SetReadDeadline(time.Now().Add(100 * time.Millisecond))
        line, err := r.ReadString('\n')

        if err == nil {
            line := strings.TrimSpace(line)
            s.reader_chan <- line
            continue loopRead
        }

        if err != nil {
            if e, ok := err.(*net.OpError); ok && e.Timeout() {

                loopWrite: for !m.IsQuitting() {
                    select {
                    case w := <- writer:
                        if len(w) > 0 {
                            fmt.Printf("[LW-SERV %d] > %s", id, w)
                            conn.Write( []byte(w) )
                        }
                        continue loopWrite
                    default:
                        continue loopRead
                    }
                }
            }
        }

        if err != nil {
            if op, ok := err.(*net.OpError); ok {
                fmt.Println("[LW-SERV] Connection error:", op.Op)
                break loopRead
            } else if err == io.EOF {
                fmt.Println("[LW-SERV] Connection closing:", err)
                break loopRead
            } else {
                fmt.Printf("[LW-SERV] Ignored error: %#v\n", err)
            }
        }
    }
    
    fmt.Println("[LW-SERV] Connection closed")
}

func HandleLwServLine(s *LwServ, line string) string {

    if line == "@I" {
        // Info command: @I\n
        fmt.Println("[LW-SERV] Info Cmd");
        // Reply: @IT<00>S<00>\n
        return fmt.Sprintf("@IT%02dS%02d\n", LW_TURNOUT_N, LW_AIU_N);
    }
    
    if len(line) == 5 && strings.HasPrefix(line, "@T") {
        // Turnout command: @T<00><N|R>\n
        buf := line[2:5]
        turnout := (uint(buf[0] - '0') << 8) + uint(buf[1] - '0')
        direction := uint8(buf[2])
        if ((direction == LW_TURNOUT_NORMAL || direction == LW_TURNOUT_REVERSE) &&
                turnout > 0 && turnout <= LW_TURNOUT_N) {
            fmt.Printf("[LW-SERV] Accepted Turnout Cmd %d = %c\n", turnout, direction);
            turnout -= 1 // cmd index is 1-based but array & relays are 0-based
            
            aiu := turnout / LW_SENSORS_PER_AIU
            turnout = turnout % LW_SENSORS_PER_AIU
            state := (s._sensors[aiu] >> turnout) & 1
            var desired uint16
            if direction == LW_TURNOUT_REVERSE {
                desired = 1
            }
            
            if state != desired {
                if direction == LW_TURNOUT_NORMAL {                
                    fmt.Printf("[LW-SERV] Simulate trigger pin %d\n", LW_RELAY_NORMAL(turnout))
                    s._sensors[aiu] = LW_CLEAR_BIT(s._sensors[aiu], turnout)
                } else {
                    fmt.Printf("[LW-SERV] Simulate trigger pin %d\n", LW_RELAY_REVERSE(turnout))
                    s._sensors[aiu] = LW_SET_BIT(s._sensors[aiu], turnout)
                }
            }
            // Reply is the same command
            return line + "\n"
        } else {
            fmt.Printf("[LW-SERV] Rejected Turnout Cmd '%s'\n", line);
        }
    }
    
    fmt.Printf("[LW-SERV] Unknown Cmd: '%s'\n", line)

    return ""
}

func LwServPollSensors(m *Model, lw *LwServ) (reply string) {
    
    loopRead: for !m.IsQuitting() {
        select {
        case s := <- lw.sensors_chan:
            index := s.Sensor
            if index < 1 || index > LW_SENSOR_N {
                panic(fmt.Errorf("Invalid LW sensor index %d [1..%d]", index, LW_SENSOR_N))
            }
            index -= 1
            aiu   := index / SENSORS_PER_AIU
            index  = index % SENSORS_PER_AIU

            if s.Active {
                lw._sensors[aiu] = LW_SET_BIT(lw._sensors[aiu], index)
                fmt.Printf("[LW-SERV] Simulate set sensor %d:%d ON\n", aiu+1, index+1)
            } else {
                lw._sensors[aiu] = LW_CLEAR_BIT(lw._sensors[aiu], index)
                fmt.Printf("[LW-SERV] Simulate set sensor %d:%d OFF\n", aiu+1, index+1)
            }
        default:
            break loopRead
        }
    }

    for aiu := 1; aiu <= LW_AIU_N; aiu++ {    
        s := lw._sensors[aiu - 1]
        if s != lw._last[aiu - 1] {
            lw._last[aiu - 1] = s
            reply += fmt.Sprintf("@S%02d%04x\n", aiu, s)
        }
    }    

    return reply
}
