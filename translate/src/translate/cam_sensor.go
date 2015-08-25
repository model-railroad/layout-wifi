package translate

import (
    "flag"
    "fmt"
    "io"
    "image"
    "image/color"
    "image/jpeg"
    "image/png"
    "mime/multipart"
    "regexp"
    "net/http"
    net_url "net/url"
    "strings"
    "strconv"
    "sync"
    "time"
)

const CAM_N_POINTS = 9
const CAM_THRESHOLD = 100
const CAM_EDGE = 50

var CAM_SERV = flag.String("cam-server", ":8088", "Camera debug server host:port")

var CAM_HOSTS = flag.String("cam-urls",
    "camera1.local:80/path.cgi?user=foo&pwd=blah," +
    "user2:pwd2@camera2.local/path.cgi",
    "Cameras user:pwd@host:port/path")

var CAM_SENSORS = flag.String("cam-sensors",
    "0,38:154,121,174,120",
    "Camera detection points: N,S:x1,y1,x2,y2...")

// Typical URLs:
// Foscam: http://foscam.ip/videostream.cgi?resolution=32&user=USER&pwd=PWD
// Edimax: http://USER:PWD@edimax.ip/mjpg/video.mjpg

//-----

var CAMERAS []*Camera

type CamSensor struct {
    sensor  int
    start   image.Point
    end     image.Point
    points  []image.Point // CAM_N_POINTS pairs x/y
    values  []int // CAM_N_POINTS current values
    empty   bool
    init    bool
}

type Camera struct {
    index   int
    url     *net_url.URL
    mutex   sync.Mutex
    last    *CamImage
    sensors []*CamSensor
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

// Based on the Converted example from http://stackoverflow.com/questions/8697095
// this remaps the image to a gray color model but _only_ for the pixels being
// acceded. Cons: No caching. Pros: good if only a few pixels are accessed.
//
// @Implements image.Image.
type CamImage struct {
    cam   *Camera
    img   image.Image
    //--model color.Model
}

func (cam *Camera) NewCamImage(source image.Image) *CamImage {
    ci := &CamImage { cam, source }
    return ci
}

func (ci *CamImage) ColorModel() color.Model {
    return color.RGBAModel     // -- ci.img.ColorModel()
}

// Return the original bounds
func (ci *CamImage) Bounds() image.Rectangle {
    return ci.img.Bounds()
}

// At() forwards the call to the original image and
// uses the color model to convert it.
func (ci *CamImage) At(x, y int) color.Color {
    for _, s := range ci.cam.sensors {
        for _, p := range s.points {
            if p.X == x && p.Y == y {
                return color.RGBA{0xFF, 0x00, 0x00, 0xFF}
            }
        }
    }

    return ci.GrayAt(x, y)
}

func (ci *CamImage) GrayAt(x, y int) color.Color {
    return color.GrayModel.Convert(ci.img.At(x,y))
}

//-----

func CamSensorDebugServer() {
    http.HandleFunc("/", CamSensorDebugHandler)

    go func() {
        if err := http.ListenAndServe(*CAM_SERV, nil /*handler*/); err != nil {
            panic(err)
        }
    }()
}

func CamSensorDebugHandler(w http.ResponseWriter, req *http.Request) {
    content := `<html><head>
        <meta http-equiv="refresh" content="5">
        <title>Translate Debug</title></head>
        <body>`

    content += fmt.Sprintf("%d Cameras:<br/>\n", len(CAMERAS))

    for _, cam := range CAMERAS {
        content += cam.DebugHtml()
    }

    content += "</body>"

    w.Header().Set("Content-Type", "text/html; charset=utf-8")
    w.WriteHeader(http.StatusOK)
    io.WriteString(w, content)
}

func CamSensorLastImageHandler(cam *Camera, w http.ResponseWriter, req *http.Request) {
    img := cam.GetLast()
    if img != nil {
        w.Header().Set("Content-Type", "image/png")
        w.WriteHeader(http.StatusOK)
        png.Encode(w, img)
        return
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
        cam.SetupSensors(*CAM_SENSORS)
        CAMERAS = append(CAMERAS, cam)
        http.HandleFunc(fmt.Sprintf("/last/%d", index),
            func(w http.ResponseWriter, r *http.Request) {
                CamSensorLastImageHandler(cam, w, r)
            })

        go func(cam *Camera) {
            for !m.IsQuitting() {
                fmt.Printf("[CAM %d] connecting to server %s\n", cam.index, cam.url.Host)

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
            fmt.Printf("[CAM %d: %s] Unexpected NextPart error: %v\n", cam.index, cam.url.Host, err)
            break loopRead
        }

        defer part.Close()

        img, err := jpeg.Decode(part)
        if err != nil {
            fmt.Printf("[CAM %d: %s] Unexpected JPEG Decode error: %v\n", cam.index, cam.url.Host, err)
            break loopRead
        }

        ci := cam.NewCamImage(img)
        cam.UpdateSensors(ci, m)
        cam.SetLast(ci)

        /* fmt.Printf("[CAM %d: %s] Decoded JPEG %d x %d\n", 
            cam.index, cam.url.Host, 
            img.Bounds().Dx(), img.Bounds().Dy())
        */
    }
    fmt.Printf("[CAM %d: %s] READ Connection closed\n", cam.index, cam.url.Host)
}

func (cam *Camera) GetLast() *CamImage {
    cam.mutex.Lock()
    defer cam.mutex.Unlock()
    return cam.last
}

func (cam *Camera) SetLast(ci *CamImage) {
    cam.mutex.Lock()
    defer cam.mutex.Unlock()
    cam.last = ci
}

func (cam *Camera) SetupSensors(config string) {
    re := regexp.MustCompile(fmt.Sprintf("([%d]),([0-9]+):([0-9]+),([0-9]+),([0-9]+),([0-9]+)", cam.index))

    match := re.FindAllStringSubmatch(config, -1 /*all*/)
    for _, result := range match {
        s := &CamSensor{}
        s.init = true

        var err error
        if s.sensor, err = strconv.Atoi(result[2]); err != nil {
            panic(err)
        }

        s.start = _parseSensorXY(result[3:5])
        s.end   = _parseSensorXY(result[5:7])

        s.points = make([]image.Point, CAM_N_POINTS)
        s.values = make([]int, CAM_N_POINTS)

        var px, py float32
        px = float32(s.start.X)
        py = float32(s.start.Y)
        var dx, dy float32
        dx = (float32(s.end.X) - px) / (CAM_N_POINTS - 1)
        dy = (float32(s.end.Y) - py) / (CAM_N_POINTS - 1)
        for i := 0; i < CAM_N_POINTS; i++ {
            s.points[i].X = int(px + float32(i) * dx)
            s.points[i].Y = int(py + float32(i) * dy)
        }

        cam.sensors = append(cam.sensors, s)
    }
}

func _parseSensorXY(str []string) (p image.Point) {
    var err error
    if p.X, err = strconv.Atoi(str[0]); err != nil {
        panic(err)
    }
    if p.Y, err = strconv.Atoi(str[1]); err != nil {
        panic(err)
    }
    return p
}

func (cam *Camera) DebugHtml() string {
    content := fmt.Sprintf(
        `<p/><a href="/last/%d">Last image from camera %d</a><br/>`,
        cam.index, cam.index)

    for index, sensor := range cam.sensors {
        content += fmt.Sprintf("[%d,%d] ", index, sensor.sensor)
        for _, v := range sensor.values {
            var color string
            if v > CAM_THRESHOLD {
                color = `style="color: red"`
            }
            content += fmt.Sprintf("<span %s>%d</span> ", color, v)
        }
        if sensor.empty {
            content += `<span style="color: green">EMPTY</span>`
        } else {
            content += `<span style="color: red">occupied</span>`
        }
        content += "<br/>"
    }

    return content
}

func (cam *Camera) UpdateSensors(img *CamImage, m *Model) {
    for _, sensor := range cam.sensors {
        v := sensor.values
        for i := 0; i < CAM_N_POINTS; i++ {
            c := img.GrayAt(sensor.points[i].X, sensor.points[i].Y)
            v[i] = int(c.(color.Gray).Y)
        }

        var is_empty bool

        const N = CAM_N_POINTS / 2
        var num_start, center, num_end int

        // count low points on start side
        for i := 0; i <= N; i++ {
            if v[i] >= CAM_THRESHOLD {
                break
            }
            if i > 0 && _abs(v[i], v[i-1]) > CAM_EDGE {
                break
            }
            num_start++
        }
        if num_start > 0 {
            // count low points on end side
            for i := CAM_N_POINTS - 1; i >= N; i-- {
                if v[i] >= CAM_THRESHOLD {
                    break
                }
                if i < CAM_N_POINTS - 1 && _abs(v[i], v[i+1]) > CAM_EDGE {
                    break
                }
                num_end++
            }

            if num_end > 0 {
                center = CAM_N_POINTS - num_start - num_end
                if center > 1 {
                    is_empty = true
                }
            }
        }

        old := sensor.empty
        sensor.empty = is_empty
        if old != is_empty || sensor.init {
            fmt.Printf("Cam Sensor changed: %v => %v [%d | %d | %d]\n", sensor.sensor, !is_empty, num_start, center, num_end)
            m.SetSensor(sensor.sensor, !is_empty)
            sensor.init = false
        }
    }
}

func _abs(a, b int) int {
    if a < b {
        return b - a
    }
    return a - b
}



