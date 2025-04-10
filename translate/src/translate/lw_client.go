package translate

import (
    "bufio"
    "flag"
    "fmt"
    "net"
    "strconv"
    "strings"
    "time"
)

var LW_CLIENT_PORT = flag.String("lw-client-port", "localhost:9090", "LayoutWifi server host:port")

type LwPendingOp struct {
    expected string
    op *TurnoutOp
    time time.Time
}

var lw_pending *LwPendingOp

//-----

func LwClient(m *Model) {
    fmt.Println("Start LW client")

    go func() {
        for !m.IsQuitting() {
            fmt.Printf("LW client connecting to LW server %s\n", *LW_CLIENT_PORT)
            conn, err := net.Dial("tcp", *LW_CLIENT_PORT)
            if err != nil {
                fmt.Printf("[LWC-READ] READ Connection error: %v", err)
                time.Sleep(5 * time.Second)

            } else {
                LwClient_Read(conn, m)
            }
        }
    }()
}

func LwClient_Read(conn net.Conn, m *Model) {
    fmt.Println("[LWC-READER] New READ connection")

    defer conn.Close()

    r := bufio.NewReader(conn)

    loopRead: for !m.IsQuitting() {

        conn.SetReadDeadline(time.Now().Add(100 * time.Millisecond))
        line, err := r.ReadString('\n')

        if err != nil {
            if e, ok := err.(*net.OpError); ok && e.Timeout() {
                LwClient_Write(conn, m)
                continue loopRead
            }
        }
        if err == nil {
            line := strings.TrimSpace(line)
            err = LwClient_ReadLine(m, line)
        }
        if err != nil {
            if op, ok := err.(*net.OpError); ok {
                fmt.Println("[LWC-READER] READ Connection error:", op.Op)
                break loopRead
            } else {
                fmt.Printf("[LWC-READER] Unexpected error: %#v\n", err)
                break loopRead
            }
        }
    }
    fmt.Println("[LWC-READER] READ Connection closed")
}

func LwClient_ReadLine(m *Model, line string) (err error) {
    fmt.Println("[LWC-READER] < ", line)
    if len(line) == 8 && strings.HasPrefix(line, "@IT") {
        // Read INFO @IT<00>S<00>
        var num_turnouts, num_sensors int
        num_turnouts, err = strconv.Atoi(line[3:5])
        num_sensors,  err = strconv.Atoi(line[6:8])
        fmt.Printf("[LWC-READER] INFO: %d Turnouts, %d AIU Sensors\n", num_turnouts, num_sensors)

    } else if len(line) == 5 && strings.HasPrefix(line, "@T") {
        // Read turnout feedbback @T<00>[N|R]
        if lw_pending != nil && lw_pending.expected == line {
            fmt.Println("[LWC-READER] Got pending expected reply:", line)
            lw_pending = nil
        }

        buf := line[2:5]
        turnout := (int(buf[0] - '0') << 8) + int(buf[1] - '0')
        direction := uint8(buf[2])
        if ((direction == LW_TURNOUT_NORMAL || direction == LW_TURNOUT_REVERSE) &&
                turnout > 0 && turnout <= LW_TURNOUT_N) {
            fmt.Printf("[LWC-READER] Feedback Turnout %d = %c\n", turnout, direction)
            m.SetTurnoutState(uint(turnout), direction == LW_TURNOUT_NORMAL)
        }
    } else if len(line) == 8 && strings.HasPrefix(line, "@S") {
        // Read Sensors @S<00><0000> with 1-base AIU and 4-hex sensor data
        var aiu int
        var data int64
        aiu,  err = strconv.Atoi(line[2:4])
        data, err = strconv.ParseInt(line[4:8], 16, 32) // parse hex int32
        if aiu > 0 && aiu <= LW_AIU_N {
            fmt.Printf("[LWC-READER] Sensors %d = %04x\n",
                aiu, uint16(data & SENSORS_MASK))
            m.SetSensors(aiu, uint16(data & SENSORS_MASK))
        }
    } else {
        fmt.Printf("[LWC-READER] Ignore unknown line: '%s'\n", line)
    }

    return err
}

func LwClient_Write(conn net.Conn, m *Model) {
    var op *TurnoutOp
    var ok bool

    if lw_pending != nil {
        if time.Now().Before(lw_pending.time) {
            // There's a pending op and it's not stale yet
            return
        }
        op = lw_pending.op
        ok = true
        lw_pending = nil
        fmt.Println("[LWC-WRITER] Retrying pending op:", op)
    } else {
        op, ok = m.GetTurnoutOp(0 * time.Millisecond)
    }

    if ok && op != nil {
        str := "R"
        if op.Normal {
            str = "N"
        }
        str = fmt.Sprintf("@T%02d%s\n", op.Address, str)
        fmt.Printf("[LWC-WRITER] > %s", str)
        _, err := conn.Write([]byte(str))

        lw_pending = &LwPendingOp{strings.TrimSpace(str), op, time.Now().Add(2 * time.Second)}
        if err != nil {
            if op, ok := err.(*net.OpError); ok {
                fmt.Println("[LWC-WRITER] WRITE Connection error:", op.Op)
                return
            } else {
                fmt.Printf("[LWC-WRITER] Unexpected error: %#v\n", err)
                return
            }
        }
    }
}

