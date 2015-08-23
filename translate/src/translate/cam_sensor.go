package translate

import (
    "flag"
    "fmt"
    "io"
    "image"
    "image/jpeg"
    "mime/multipart"
    "net/http"
    net_url "net/url"
    "strings"
    "sync"
    "time"
)

var CAM_SERV = flag.String("cam-server", ":8088", "Camera debug server host:port")

var CAM_HOSTS = flag.String("cam-urls",
    "camera1.local:80/path.cgi?user=foo&pwd=blah," +
    "user2:pwd2@camera2.local/path.cgi", 
    "Cameras user:pwd@host:port/path")

// Typical URLs:
// Foscam: http://foscam.ip/videostream.cgi?resolution=32&user=USER&pwd=PWD
// Edimax: http://USER:PWD@edimax.ip/mjpg/video.mjpg

//-----

var CAMERAS []*Camera

type Camera struct {
    index   int
    url     *net_url.URL
    mutex   sync.Mutex
    last    image.Image
}

func NewCamera(index int, url_str string) *Camera {
    u, err := net_url.Parse(url_str)
    if err != nil {
        panic(err)
    }
    cam := &Camera{}
    cam.index = index
    cam.url = u
    return cam
}

//-----

func CamSensorDebugServer() {
    http.HandleFunc("/debug", CamSensorDebugHandler)

    if err := http.ListenAndServe(*CAM_SERV, nil /*handler*/); err != nil {
        panic(err)
    }
}

func CamSensorDebugHandler(w http.ResponseWriter, req *http.Request) {
    if len(CAMERAS) > 0 {
        cam := CAMERAS[0]
        img := cam.GetLast()
        if img != nil {
            w.Header().Set("Content-Type", "image/jpeg")
            w.WriteHeader(http.StatusOK)
            jpeg.Encode(w, img, nil /*Options*/)
            return
        }
    }
    
    w.Header().Set("Content-Type", "text/plain; charset=utf-8")
    w.WriteHeader(http.StatusOK)
    io.WriteString(w, "No image available.\n")
}

//-----

func CamSensorClient(m *Model) {
    fmt.Println("Start Cam-Sensor client")

    urls := strings.Split(*CAM_HOSTS, ",")

    for index, url := range urls {
        cam := NewCamera(index, url)
        CAMERAS = append(CAMERAS, cam)

        go func(cam *Camera) {
            for !m.IsQuitting() {
                fmt.Printf("[CAM %d] connecting to server %s\n", cam.index, cam.url.Host)

                /*
                req, err := http.NewRequest("GET", cam.url, nil)
                if err != nil {
                    if cam.url.User != nil {
                        req.SetBasicAuth(cam.url.User.Username(), cam.url.User.Password())
                    }
                    client := &http.Client{}
                    resp, err := client.Do(req)
                }
                */
                resp, err := http.Get(cam.url.String())

                if err != nil {
                    fmt.Printf("[CAM %d: %s] READ Connection error: %v\n", cam.index, cam.url.Host, err)
                    time.Sleep(5 * time.Second)

                } else {
                    cam.CamClient(resp.Body, m)
                }
            }
        }(cam)
    }
}

func (cam *Camera) CamClient(stream io.ReadCloser, m *Model) {
    fmt.Printf("[CAM %d: %s] New READ connection\n", cam.index, cam.url.Host)
    
    boundary := "ipcamera" // TODO
    
    defer stream.Close()
    reader := multipart.NewReader(stream, boundary)
    
    loopRead: for !m.IsQuitting() {
        part, err := reader.NextPart()
        if err == io.EOF {
            break loopRead
        } else if err != nil {
            panic(err)
        }
        
        //-- bytes, err := ioutil.ReadAll(part)
        defer part.Close()
        
        img, err := jpeg.Decode(part)
        if err != nil {
            panic(err)
        }
        
        cam.SetLast(img)
        
        fmt.Printf("[CAM %d: %s] Decoded JPEG %d x %d\n", 
            cam.index, cam.url.Host, 
            img.Bounds().Dx(), img.Bounds().Dy())
    }
    fmt.Printf("[CAM %d: %s] READ Connection closed\n", cam.index, cam.url.Host)
}

func (cam *Camera) GetLast() image.Image {
    cam.mutex.Lock()
    defer cam.mutex.Unlock()
    return cam.last
}

func (cam *Camera) SetLast(img image.Image) {
    cam.mutex.Lock()
    defer cam.mutex.Unlock()
    cam.last = img
}
