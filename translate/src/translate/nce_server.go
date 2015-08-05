package translate

import (
    "flag"
    "fmt"
    "net"
)

var NCE_PORT = flag.String("nce-port", ":8080", "NCE server host:port")

const NCE_GET_VERSION     = 0xAA
const NCE_GET_AIU_SENSORS = 0x8A
const NCE_READ_TURNOUTS   = 0x8F
const NCE_TRIGGER_ACC     = 0xAD
const NCE_READ_RAM        = 0x9D

func NceServer(m *Model) {
    fmt.Println("Start NCE server")

    listener, err := net.Listen("tcp", *NCE_PORT)
    if err != nil {
        panic(err)
    }
    
    go func(listener net.Listener) {
        defer listener.Close()
        
        for !m.IsQuitting() {
            conn, err := listener.Accept()
            if err != nil {
                panic(err)
            }
            go HandleNceConn(m, conn)
        }
    }(listener)
}

func HandleNceConn(m *Model, conn net.Conn) {
    fmt.Println("[NCE] New connection")

    defer conn.Close()

    sensors := make([]uint16, MAX_AIUS)
    buf := make([]byte, 16)

    loopRead: for !m.IsQuitting() {
        n, err := conn.Read(buf[0:1])

        if n == 1 && err == nil {
            cmd := buf[0]

            // TODO limit with if-verbose flag
            fmt.Printf("[NCE] Command: 0x%02x\n", cmd)
            
            switch cmd {
            case NCE_GET_VERSION:
                // Op: Get version.
                // Args: None
                // Reply: version 6.3.8
                fmt.Println("[NCE] > Version")
                buf[0] = 6
                buf[1] = 3
                buf[2] = 8
                n, err = conn.Write(buf[0:3])
            
            case NCE_GET_AIU_SENSORS:            
                // Op: AIU polling
                // Args: 1 byte (AIU number)
                // Reply: returns 4 bytes (sensor BE, mask BE, 14 bits max)
                n, err := conn.Read(buf[0:1])
                aiu := int(buf[0])
                if n == 1 && err == nil && aiu >= 1 && aiu <= MAX_AIUS {
                    s := m.GetSensors(aiu)
                    buf[0] = byte((s >> 8) & 0x0FF)
                    buf[1] = byte( s       & 0x0FF)

                    mask := s ^ sensors[aiu - 1]
                    sensors[aiu - 1] = s

                    buf[2] = byte((mask >> 8) & 0x0FF)
                    buf[3] = byte( mask       & 0x0FF)

                    // TODO limit with if-verbose flag
                    fmt.Printf("[NCE] > Poll AIU[%d] = %04x ^ %04x\n", aiu, s, mask)
                    
                    n, err = conn.Write(buf[0:4])
                } else {
                    fmt.Printf("[NCE] > Invalid Poll AIU [%d], n=%d, err=%v\n", aiu, n, err)
                }
                
            case NCE_TRIGGER_ACC:
                // Op: Trigger accessories (i.e. turnouts)
                // Args: 4 bytes (2 for address big endian, 1 op: 3=normal/on, 4=reverse/off, 1 byte=0)
                // Reply: 1 byte "!"
                n, err := conn.Read(buf[0:4])
                addr := (int(buf[0]) << 8) + int(buf[1])
                op   := int(buf[2])
                if n == 4 && err == nil && addr >= 1 && addr <= MAX_TURNOUTS && (op == 3 || op == 4) {
                    fmt.Printf("[NCE] > Trigger Acc [%04x], op=%d\n", addr, op)

                    m.SendTurnoutOp( &TurnoutOp{addr, op == 3} )
                    
                    buf[0] = '!'
                    n, err = conn.Write(buf[0:1])
                } else {
                    fmt.Printf("[NCE] > Invalid Trigger Acc [%04x], op=%d, n=%d, err=%v\n", addr, op, n, err)
                }
                
            case NCE_READ_RAM:
                // Op: Read one byte from RAM
                // Args: 2 bytes (address, big endian)
                // Reply: 1 byte
                n, err := conn.Read(buf[0:2])
                addr := (int(buf[0]) << 8) + int(buf[1])
                if n == 2 && err == nil && addr >= 0 {
                    // Mock: we ignore the address and just return 0 all the time.
                    fmt.Printf("[NCE] > Read RAM [%04x]\n", addr)
                    buf[0] = 0
                    n, err = conn.Write(buf[0:1])
                } else {
                    fmt.Printf("[NCE] > Invalid Read RAM [%04x], n=%d, err=%v\n", addr, n, err)
                }

            case NCE_READ_TURNOUTS:
                // Op: Read 16-bytes from the address specified
                // Args: 2 bytes (address, big endian)
                // Reply: 16 bytes which are turnout feedback state,
                // 1 bit per turnout "sensor", little endian.
                n, err := conn.Read(buf[0:2])
                addr := (int(buf[0]) << 8) + int(buf[1])
                if n == 2 && err == nil && addr >= 0 {
                    // Mock: we ignore the address and just return all turnout states
                    states := m.GetTurnoutStates()
                    for i := 0; i < 4; i++ {
                        buf[i] = byte(states & 0x0FF)
                        states = states >> 8
                    }
                    for i := 4; i < 16; i++ {
                        buf[i] = 0
                    }
                    
                    fmt.Printf("[NCE] > Read Turnouts [%04x] %v\n", addr, buf)
                    n, err = conn.Write(buf[0:16])
                } else {
                    fmt.Printf("[NCE] > Invalid Read Turnouts [%04x], n=%d, err=%v\n", addr, n, err)
                }
                
            default:
                fmt.Printf("[NCE] > Ignored command 0x%02x\n", cmd)
            }
        }

        if err != nil {
            if op, ok := err.(*net.OpError); ok {
                fmt.Println("[NCE] Connection error:", op.Op)
                break loopRead
            } else {
                panic(err)
            }
        }
    }
    fmt.Println("[NCE] Connection closed")
}
