package translate

import (
    "flag"
    "fmt"
    "net"
    "strings"
    "time"
)

var CAM_HOSTS = flag.String("cam-hosts", "camera1.local:80,camera2.local:80", "Cameras host:port")

type Camera struct {
    index   int
    host    string
}


//-----


func CamSensorClient(m *Model) {
    fmt.Println("Start cam-sensor client")

    hosts := strings.Split(*CAM_HOSTS, ",")

    for index, host := range hosts {
        cam := &Camera { index, host }

        go func(cam *Camera) {
            for !m.IsQuitting() {
                fmt.Printf("[CAM %d] connecting to server %s\n", cam.index, cam.host)
                conn, err := net.Dial("tcp", cam.host)
                if err != nil {
                    fmt.Printf("[CAM %d] READ Connection error: %v", cam.index, err)
                    time.Sleep(5 * time.Second)

                } else {
                    cam.CamClient(conn, m)
                }
            }
        }(cam)
    }

}

func (cam *Camera) CamClient(conn net.Conn, m *Model) {
    fmt.Println("[LWC-READER] New READ connection")
    defer conn.Close()
    loopRead: for !m.IsQuitting() {
        break loopRead
    }
}
