package translate

import (
    "bufio"
    "flag"
    "fmt"
    "io"
    "os"
    "os/signal"
    "os/user"
    "path/filepath"
    "strings"
    "strconv"
    "syscall"
)

var VERBOSE = flag.Bool("verbose", false, "Verbose output")
var LW_SERV = flag.Bool("simulate", false, "Simulate LayoutWifi server")
var ENABLE_CAM_SERV = flag.Bool("enable-cam-server", true, "Enable camera server")
var CONFIG_FILE string
var CONFIG = NewConfig()

func init() {
    filename := "~/.translaterc"
    if usr, err := user.Current(); err == nil {
        filename = filepath.Join(usr.HomeDir, filename[2:])
    }
    flag.StringVar(&CONFIG_FILE, "config", filename, "Config file to read")
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

        case *LW_SERV && strings.HasPrefix(str, "on"):
            fields := strings.Fields(str)
            if index, err :=  strconv.Atoi(fields[1]); err == nil {
                sensors_chan <- LwSensor { uint(index), true }
            }
        case *LW_SERV && strings.HasPrefix(str, "off"):
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
    CONFIG.ReadFile(CONFIG_FILE)
    CONFIG.UpdateFlags(flag.CommandLine)

    model := NewModel()
    SetupSignal(model)
    NceServer(model)
    SrcpServer(model)

    if (*ENABLE_CAM_SERV) {
        CamSensorClient(model)
        CamSensorDebugServer()
    }

    var sensors_chan chan LwSensor
    if (*LW_SERV) {
        // Simulate DigiX server
        sensors_chan = LwServer(model)
    }

    LwClient(model)
    TerminalLoop(model, sensors_chan)
}

