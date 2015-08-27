package translate

import (
    "bufio"
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

var CAM_OFFSETS = flag.String("cam-offsets", "0:0,0 1:0,0", "Camera sensor offsets")



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
    min     int
    max     int
    threshold int
}

type Camera struct {
    index   int
    url     *net_url.URL
    boundary string
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
}

func (cam *Camera) NewCamImage(source image.Image) *CamImage {
    ci := &CamImage { cam, source }
    return ci
}

func (ci *CamImage) ColorModel() color.Model {
    return color.RGBAModel
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
    http.HandleFunc("/reload", CamSensorReloadHandler)

    go func() {
        if err := http.ListenAndServe(*CAM_SERV, nil /*handler*/); err != nil {
            panic(err)
        }
    }()
}

func CamSensorDebugHandler(w http.ResponseWriter, req *http.Request) {
    content := `<html><head>
        <title>Translate Debug</title></head>
        <body>`

    content += fmt.Sprintf("%d Cameras:<p/>\n", len(CAMERAS))

    content += "<a href='/reload'>Reload config</a><p/>\n"

    cams := ""
    for _, cam := range CAMERAS {
        if cams != "" {
            cams += ", "
        }
        cams += fmt.Sprintf("{ index: %d }", cam.index)
        content += fmt.Sprintf("<div id='cam%d'></div><p/>\n", cam.index)
    }

    content += `
<script src="//ajax.googleapis.com/ajax/libs/jquery/1.11.1/jquery.min.js"></script>
<script>
cams = [` + cams + `];
for (c = 0; c < cams.length; c++) {
    var cam = cams[c]
    cam.xh = new XMLHttpRequest();
    cam.xh.onreadystatechange = function(cam) {
        return function() {
            if (cam.xh.readyState == 4 && cam.xh.status == 200) {
                var div = $("#cam" + cam.index);
                div.html(cam.xh.responseText);
            }
        }
    }(cam);
}

setInterval(function() {
    for (c = 0; c < cams.length; c++) {
        var cam = cams[c]
        var url = "/state/" + cam.index;
        cam.xh.open("GET", url, true);
        cam.xh.send();
    }
}, 2000);
</script>

</body>
`

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

func CamSensorReloadHandler(w http.ResponseWriter, req *http.Request) {
    var config = NewConfig()
    config.ReadFile(CONFIG_FILE)
    for _, cam := range CAMERAS {
        cam.SetupSensors(config.Get("cam-sensors", ""), config.Get("cam-offsets", ""))
    }
    content := `<html>
<header>
<title>Reload</title>
<meta http-equiv="refresh" content="1;/">
</header>
<body>
Reload complete.
</body>`
    w.Header().Set("Content-Type", "text/html; charset=utf-8")
    w.WriteHeader(http.StatusOK)
    io.WriteString(w, content)
}

func CamSensorClient(m *Model) {
    fmt.Println("Start Cam-Sensor client")

    urls := strings.Split(*CAM_HOSTS, ",")

    for index, url := range urls {
        cam := NewCamera(index, url)
        cam.SetupSensors(*CAM_SENSORS, *CAM_OFFSETS)
        CAMERAS = append(CAMERAS, cam)
        http.HandleFunc(fmt.Sprintf("/last/%d", index),
            func(w http.ResponseWriter, r *http.Request) {
                CamSensorLastImageHandler(cam, w, r)
            })
        http.HandleFunc(fmt.Sprintf("/state/%d", index),
            func(w http.ResponseWriter, r *http.Request) {
                io.WriteString(w, cam.DebugHtml())
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

    defer stream.Close()

    if cam.boundary == "" {
        // First figure what is the multipart boundary. Simply read
        // the stream til we find a line starting with --
        r := bufio.NewReader(stream)
        for !m.IsQuitting() {
            str, err := r.ReadString('\n')
            if err != nil {
                fmt.Printf("[CAM %d: %s] Unexpected boundary error: %v\n", cam.index, cam.url.Host, err)
                break
            }
            if strings.HasPrefix(str, "--") {
                str = strings.TrimSpace(str[2:])
                if strings.HasSuffix(str, "--") {
                    str = str[:len(str) - 2]
                }
                cam.boundary = str
                fmt.Printf("[CAM %d: %s] Multipart Boundary: %s\n", cam.index, cam.url.Host, str)
                break
            }
        }

        return
    }

    reader := multipart.NewReader(stream, cam.boundary)

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

func (cam *Camera) SetupSensors(config, offsets string) {
    var err error
    re := regexp.MustCompile(fmt.Sprintf("([%d]):(-?[0-9]+),(-?[0-9]+)", cam.index))
    var offset image.Point
    if match := re.FindStringSubmatch(offsets); match != nil {
        offset = _parsePoint(match[2:])
    }
    fmt.Printf("[CAM %d: %s] Setup sensors with offset %v\n", cam.index, cam.url.Host, offset)

    re = regexp.MustCompile(fmt.Sprintf("([%d]),([0-9]+):([0-9]+),([0-9]+),([0-9]+),([0-9]+)", cam.index))

    match := re.FindAllStringSubmatch(config, -1 /*all*/)
    if cam.sensors == nil {
        cam.sensors = make([]*CamSensor, len(match))
        for i := range match {
            cam.sensors[i] = &CamSensor{}
        }
    }
    for index, result := range match {
        s := cam.sensors[index]
        s.init = true
        s.min = 0
        s.max = 255
        s.threshold = (s.min + s.max) / 2

        if s.sensor, err = strconv.Atoi(result[2]); err != nil {
            panic(err)
        }

        s.start = _offset(_parsePoint(result[3:5]), offset)
        s.end   = _offset(_parsePoint(result[5:7]), offset)

        if s.points == nil {
            s.points = make([]image.Point, CAM_N_POINTS)
            s.values = make([]int, CAM_N_POINTS)
        }

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
    }
}

func _parsePoint(str []string) (p image.Point) {
    var err error
    if p.X, err = strconv.Atoi(str[0]); err != nil {
        panic(err)
    }
    if p.Y, err = strconv.Atoi(str[1]); err != nil {
        panic(err)
    }
    return p
}

func _offset(p, o image.Point) image.Point {
    return image.Point { p.X + o.X, p.Y + o.Y }
}

func (cam *Camera) DebugHtml() string {
    content := fmt.Sprintf(
        `<a href="/last/%d">Last image from camera %d</a><br/>`,
        cam.index, cam.index)

    for index, s := range cam.sensors {
        content += fmt.Sprintf("[%d,%d] [%d | %d | %d] ", index, s.sensor, s.min, s.threshold, s.max)
        for _, v := range s.values {
            var color string
            if v > s.threshold {
                color = `style="color: red"`
            }
            content += fmt.Sprintf("<span %s>%d</span> ", color, v)
        }
        if s.empty {
            content += `<span style="color: green">EMPTY</span>`
        } else {
            content += `<span style="color: red">occupied</span>`
        }
        content += "<br/>"
    }

    return content
}

func (cam *Camera) UpdateSensors(img *CamImage, m *Model) {
    for _, s := range cam.sensors {
        v := s.values
        min := 255
        max := 0
        for i := 0; i < CAM_N_POINTS; i++ {
            c := img.GrayAt(s.points[i].X, s.points[i].Y)
            v[i] = int(c.(color.Gray).Y)
            if v[i] < min {
                min = v[i]
            }
            if v[i] > max {
                max = v[i]
            }
        }

        const T = 10
        const T1 = T-1
        min = (min + T1 * s.min) / T
        max = (max + T1 * s.max) / T
        if max < CAM_THRESHOLD {
            max = CAM_THRESHOLD
        }
        threshold := (min + 2 * max) / 3
        s.min = min
        s.max = max
        s.threshold = threshold

        var is_empty bool

        const N = CAM_N_POINTS / 2
        var num_start, center, num_end int

        // count low points on start side
        for i := 0; i <= N; i++ {
            if v[i] >= threshold {
                break
            }
            /* if i > 0 && _abs(v[i], v[i-1]) > CAM_EDGE {
                break
            } */
            num_start++
        }
        if num_start > 0 {
            // count low points on end side
            for i := CAM_N_POINTS - 1; i >= N; i-- {
                if v[i] >= threshold {
                    break
                }
                /* if i < CAM_N_POINTS - 1 && _abs(v[i], v[i+1]) > CAM_EDGE {
                    break
                } */
                num_end++
            }

            if num_end > 0 {
                center = CAM_N_POINTS - num_start - num_end
                if center > 1 {
                    is_empty = true
                }
            }
        }

        old := s.empty
        s.empty = is_empty
        if old != is_empty || s.init {
            fmt.Printf("Cam Sensor changed: %v => %v [%d | %d | %d] %v\n", s.sensor, !is_empty, num_start, center, num_end, v)
            m.SetSensor(s.sensor, !is_empty)
            s.init = false
        }
    }
}

func _abs(a, b int) int {
    if a < b {
        return b - a
    }
    return a - b
}



