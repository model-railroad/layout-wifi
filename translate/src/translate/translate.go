package translate

import (
    "bufio"
    "flag"
    "fmt"
    "io"
    "os"
    "os/signal"
    "strings"
    "strconv"
    "syscall"
)

var LW_SERV bool

func init() {
    flag.BoolVar(&LW_SERV, "simulate", false, "Simulate LayoutWifi server")
}

// -----

func SetupSignal(m *Model) {
    c := make(chan os.Signal, 1)
    signal.Notify(c, os.Interrupt)  // Ctrl-C
    signal.Notify(c, syscall.SIGTERM) // kill -9
    go func(m *Model) {
        <-c
        fmt.Println("Caught Ctrl-C")
        m.SetQuitting()
    }(m)
}

func ReadLine(in io.ReadWriter) string {
    fmt.Print("> ")
    b := bufio.NewReader(in)
    line, _ /*hasMore*/, err := b.ReadLine()
    if err != nil {
        panic(err)
    }
    return string(line)
}

func TerminalLoop(m *Model, sensors_chan chan<- LwSensor) {
    fmt.Println("Enter terminal")

    for !m.IsQuitting() {
        str := ReadLine(os.Stdin)

        switch {
        case str == "quit" || str == "q":
            m.SetQuitting()

        case LW_SERV && strings.HasPrefix(str, "on"):
            fields := strings.Fields(str)
            if index, err :=  strconv.Atoi(fields[1]); err == nil {
                sensors_chan <- LwSensor { uint(index), true }
            }
        case LW_SERV && strings.HasPrefix(str, "off"):
            fields := strings.Fields(str)
            if index, err :=  strconv.Atoi(fields[1]); err == nil {
                sensors_chan <- LwSensor { uint(index), false }
            }
        
        default:
            fmt.Println("Unknown command. Use quit or q.")
        }
    }
    fmt.Println("Exiting")
}

func Main() {
    flag.Parse()
    model := NewModel()
    SetupSignal(model)
    NceServer(model)
    SrcpServer(model)
    
    var sensors_chan chan LwSensor
    if (LW_SERV) {
        // Simulate DigiX server
        sensors_chan = LwServer(model)
    }

    LwClient(model)
    TerminalLoop(model, sensors_chan)
}

