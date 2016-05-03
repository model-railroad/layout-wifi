package translate

import (
    "bufio"
    "bytes"
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
    "net"
)

const CAM_N_POINTS = 12
const CAM_BASE_MAX = 100
const CAM_THRESHOLD_FACTOR = 0.70
const CAM_N_ACTIVE = 3
const CAM_MIN_MAX_DELTA = 50
const CAM_TIMER_IGNORE = time.Duration(10 * time.Second)

var CAM_SERV = flag.String("cam-server", ":8088", "Camera debug server host:port")

var SENSORS_SERV = flag.String("sensor-server", ":8090", "Sensors server host:port")

var CAM_HOSTS = flag.String("cam-urls",
    "camera1.local:80/path.cgi?user=foo&pwd=blah," +
    "user2:pwd2@camera2.local/path.cgi",
    "Cameras user:pwd@host:port/path")

var CAM_SENSORS = flag.String("cam-sensors",
    "0,38:154,121,174,120",
    "Camera detection points: N,S:x1,y1,x2,y2...")

var CAM_OFFSETS = flag.String("cam-offsets", "0:0,0 1:0,0", "Camera sensor offsets")

var CAM_TIMER_SENSORS = flag.String("cam-timer-sensors",
    "0,1,0-1",
    "Sensors to use as loop or start-stop timers. Empty to disable.")


// Typical URLs:
// Foscam: http://foscam.ip/videostream.cgi?resolution=32&user=USER&pwd=PWD
// Edimax: http://USER:PWD@edimax.ip/mjpg/video.mjpg
// D-Link: http://USER:PWD@dlink.ip/mjepg.cgi

//-----

var CAMERAS []*Camera
var CAM_TIMERS []*CamTimer
var CAM_SENSORS_CHANNELS CamSensorsChannels

type CamTimerSensor struct {
    sensor       int
    trigger_time time.Time
    ignore_time  time.Time
}

type CamTimer struct {
    start       *CamTimerSensor
    stop        *CamTimerSensor
    durations   []time.Duration
    mutex       sync.Mutex
}

type CamSensor struct {
    sensor  int
    start   image.Point
    end     image.Point
    points  []image.Point   // CAM_N_POINTS pairs x/y
    values  []int           // CAM_N_POINTS current values
    empty   bool
    init    bool
    min     int
    max     int
    threshold int
    timers  []*CamTimer
}

type CamSensorsChannels struct {
    mutex sync.Mutex
    channels map[chan string]bool
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
        for i, p := range s.points {
            if p.X == x && p.Y == y {
                if s.empty && s.values[i] > s.threshold {
                    return color.RGBA{0x00, 0xFF, 0x00, 0xFF}
                } else {
                    return color.RGBA{0xFF, 0x00, 0x00, 0xFF}
                }
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

func _getPort(s string) string {
    i := strings.LastIndex(s, ":")
    if i != -1 {
        return s[i+1:]
    }
    return s
}

func CamSensorDebugHandler(w http.ResponseWriter, req *http.Request) {
    content := `<html><head>
        <title>Translate Server Status</title>
        <style>
        .overview { font-family: monospace; }
        </style>
        </head>
        <body><h2>Translate Server</h2>
        <table width="100%"><tr><td><ul>`

    content += fmt.Sprintf("<li>SRCP Server: %s</li>\n", _getPort(*SRCP_PORT))
    content += fmt.Sprintf("<li>NCE Server: %s</li>\n", _getPort(*NCE_PORT))
    content += fmt.Sprintf("<li>Status Server: %s</li>\n", _getPort(*CAM_SERV))
    content += "</ul></td><td><ul>\n"
    content += fmt.Sprintf("<li>LayoutWifi Arduino: %s</li>\n", _getPort(*LW_CLIENT_PORT))
    if *LW_SERV {
        content += fmt.Sprintf("<li>LayoutWifi Simulator: %s</li>\n", _getPort(*LW_SERV_PORT))
    } else {
        content += "<li>LayoutWifi Simulator: stopped</li>\n"
    }
    content += fmt.Sprintf("<li>Sensor Cameras: %d</li>\n", len(CAMERAS))

    content += "</ul></td></tr></table>\n"

    content += "<a href='/reload'>Reload config</a><p/>\n"
    content += "<table><tr>\n"

    cams := ""
    timers := ""
    for _, cam := range CAMERAS {
        if cams != "" {
            cams += ", "
        }
        cams += fmt.Sprintf("{ index: %d }", cam.index)
        content += fmt.Sprintf("<td style='vertical-align: top'><div id='cam%d' class='overview'></div></td>\n", cam.index)
    }
    content += "</tr><tr>\n"
    for _, cam := range CAMERAS {
        content += strings.Replace(
        `<td>
        Camera @@:
        <a href="/last/@@" target="_blank">Open in new tab</a> | <a href="#" id="refresh@@">Refresh</a><br/>
        <img id="img@@" src="/last/@@" />
        </td>`,
        "@@", strconv.Itoa(cam.index), -1)
    }
    content += "</tr></table>\n"
    content += "<p/> <a id='timer' href='#timer'>Timers</a> | <a id='reset_timer' href='#'>Reset</a> <br/>"
    content += "<table width='100%'><tr>"
    for index, _ := range CAM_TIMERS {
        index += 1
        content += fmt.Sprintf("<td><div id='timer%d'></td>", index)
        if timers != "" {
            timers += ", "
        }
        timers += fmt.Sprintf("{ index: %d }", index)
    }
    content += "</tr></table><p/>\n"

    content += `
<script src="//ajax.googleapis.com/ajax/libs/jquery/1.11.1/jquery.min.js"></script>
<script>
cams = [` + cams + `];
for (c = 0; c < cams.length; c++) {
    var cam = cams[c];
    cam.overview = "";
    cam.xh = new XMLHttpRequest();
    cam.xh.onreadystatechange = function(cam) {
        return function() {
            if (cam.xh.readyState == 4 && cam.xh.status == 200) {
                var div = $("#cam" + cam.index);
                div.html(cam.xh.responseText);
                var o = cam.xh.responseText.match(/\|[-o]+\|/);
                if (o != null) {
                    o = o[0].trim();
                    if (o != cam.overview) {
                        console.log("Cam " + cam.index + " changed " + cam.overview + " => " + o);
                        cam.overview = o;
                        cam.img_refresher();
                    }
                }
            }
        }
    }(cam);

    cam.img_refresher = function(cam) {
        return function() {
            var img = $("#img" + cam.index);
            img.attr("src", "/last/" + cam.index + "?" + new Date().getTime());
            console.log("Update #img" + cam.index + " to " + img.attr("src"));
            return false;
        }
    }(cam);
    $("#refresh" + cam.index).click(cam.img_refresher);
}

$("#reset_timer").click(function() {
    $.ajax({
        type: "POST",
        url: "/timer/reset"
    });
    return false;
});

timers = [` + timers + `];
for (t = 0; t < timers.length; t++) {
    var timer = timers[t];
    timer.xh = new XMLHttpRequest();
    timer.xh.onreadystatechange = function(timer) {
        return function() {
            if (timer.xh.readyState == 4 && timer.xh.status == 200) {
                var div = $("#timer" + timer.index);
                div.html(timer.xh.responseText);
            }
        }
    }(timer);
}

setInterval(function() {
    for (c = 0; c < cams.length; c++) {
        var cam = cams[c]
        var url = "/state/" + cam.index;
        cam.xh.open("GET", url, true);
        cam.xh.send();
    }
    for (t = 0; t < timers.length; t++) {
        var timer = timers[t];
        timer.xh.open("GET", "/timer/" + timer.index, true);
        timer.xh.send();
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

func CamSensorTimerHandler(timer *CamTimer, w http.ResponseWriter, req *http.Request) {
    content := fmt.Sprintf("<html>Timer: %d", timer.start.sensor)
    if timer.stop != nil {
        content += fmt.Sprintf("-%d", timer.stop.sensor)
    }
    content += "<br/><hr/><br/>"

    timer.mutex.Lock()
    defer timer.mutex.Unlock()
    for i, dur := range timer.durations {
        content += fmt.Sprintf("%d: %v <br/>\n", i, dur)
    }
    content += "</html>"

    w.Header().Set("Content-Type", "text/plain; charset=utf-8")
    w.WriteHeader(http.StatusOK)
    io.WriteString(w, content)
}

func CamSensorTimerResetHandler(w http.ResponseWriter, req *http.Request) {
    for _, t := range CAM_TIMERS {
        go func(timer *CamTimer) {
            timer.mutex.Lock()
            defer timer.mutex.Unlock()
            timer.durations = nil  // clear the slice. Range and append work fine with nil.
        }(t)
    }

    w.Header().Set("Content-Type", "text/plain; charset=utf-8")
    w.WriteHeader(http.StatusOK)
    io.WriteString(w, "OK")
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

                } else if strings.HasPrefix(cam.url.Host, "dlink") {
                    cam.CamClientDlink(resp.Body, m)
                } else {
                    cam.CamClient(resp.Body, m)
                }
            }
        }(cam)
    }

    if *CAM_TIMER_SENSORS != "" {
        http.HandleFunc("/timer/reset", CamSensorTimerResetHandler)

        for _, t := range strings.Split(*CAM_TIMER_SENSORS, ",") {
            timer := NewCamTimer(t)
            if timer != nil {
                CAM_TIMERS = append(CAM_TIMERS, timer)
                index := len(CAM_TIMERS)
                http.HandleFunc(fmt.Sprintf("/timer/%d", index),
                    func(w http.ResponseWriter, r *http.Request) {
                        CamSensorTimerHandler(timer, w, r)
                    })
            }
        }
    }

   runSensorsServer(m)
}

func NewCamTimer(sensor_info string) *CamTimer {
    var sensor1, sensor2 int
    var err error

    if strings.Contains(sensor_info, "-") {
        // Start-stop dual sensor
        segments := strings.Split(sensor_info, "-")
        sensor1, err = strconv.Atoi(segments[0])
        if err == nil {
           sensor2, err = strconv.Atoi(segments[1])
        }
    } else {
        // Single sensor
        sensor1, err = strconv.Atoi(sensor_info)
        sensor2 = 0
    }

    if err != nil {
        fmt.Printf("[CAM] Unexpected timer sensor definition: %v\n", sensor_info, err)
        return nil
    }

    timer := &CamTimer{}
    timer.start = &CamTimerSensor{}
    timer.start.sensor = sensor1
    cs1 := GetCamSensor(sensor1)
    cs1.timers = append(cs1.timers, timer)

    if sensor2 > 0 {
        timer.stop = &CamTimerSensor{}
        timer.stop.sensor = sensor2
        cs2 := GetCamSensor(sensor2)
        cs2.timers = append(cs2.timers, timer)
    }

    return timer
}

func GetCamSensor(sensor_index int) *CamSensor {
    for _, cam := range CAMERAS {
        for _, sensor := range cam.sensors {
            if sensor.sensor == sensor_index {
                return sensor
            }
        }
    }
    return nil
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

// The dlink multipart mjpeg stream doesn't start with a --boundary and
// this doesn't work with the regular multipart reader.
func (cam *Camera) CamClientDlink(stream io.ReadCloser, m *Model) {
    fmt.Printf("[CAM %d: %s] New DLINK READ connection\n", cam.index, cam.url.Host)

    defer stream.Close()

    r := bufio.NewReader(stream)

    loopRead: for !m.IsQuitting() {
        // Find Content-length header
        contentLength := 0
        loopContentLength: for !m.IsQuitting() {
            str, err := r.ReadString('\n')
            if err == nil && strings.HasPrefix(str, "Content-length: ") {
                contentLength, err = strconv.Atoi(strings.TrimSpace(str[16:]))
                break loopContentLength
            }
            if err != nil {
                fmt.Printf("[CAM %d: %s] Dlink Content-length error: %v\n", cam.index, cam.url.Host, err)
                break loopRead
            }
        }

        // Skip read of header till empty line delimiter
        loopHeader: for !m.IsQuitting() {
            str, err := r.ReadString('\n')
            if err == nil && str == "\r\n" {
                break loopHeader
            }
            if err != nil {
                fmt.Printf("[CAM %d: %s] Dlink header error: %v\n", cam.index, cam.url.Host, err)
                break loopRead
            }
        }

        dest := make([]byte, contentLength)

        for pos := 0; pos < contentLength; {
            n, err := r.Read(dest[pos:])
            if err != nil {
                fmt.Printf("[CAM %d: %s] Dlink data error: %v\n", cam.index, cam.url.Host, err)
                break loopRead
            }
            pos += n
        }

        r2 := bytes.NewReader(dest)
        img, err := jpeg.Decode(r2)
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

    re = regexp.MustCompile(
        fmt.Sprintf("([%d]),([0-9]+):([0-9]+),([0-9]+),([0-9]+),([0-9]+)", cam.index))

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
    content := "<br/>"
    overview := "|"

    for index, s := range cam.sensors {
        content += fmt.Sprintf("[%2d,%3d] [%02X | %02X | %02X] ",
            index, s.sensor, s.min, s.threshold, s.max)
        for _, v := range s.values {
            var color string
            if v > s.threshold {
                color = `style="color: red"`
            }
            content += fmt.Sprintf("<span %s>%02X</span> ", color, v)
        }
        if s.empty {
            content += `<span style="color: green">EMPTY</span>`
            overview += "-"
        } else {
            content += `<span style="color: red">occupied</span>`
            overview += "o"
        }
        content += "<br/>"
    }
    overview += "|\n"

    return overview + content
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
        if max < CAM_BASE_MAX {
            max = CAM_BASE_MAX
        }
        threshold := int(float64(min) + CAM_THRESHOLD_FACTOR * float64(max - min))
        s.min = min
        s.max = max
        s.threshold = threshold

        var is_empty bool

        const N = CAM_N_POINTS / 2
        var num_start, center, num_end int

        if _abs(min, max) >= CAM_MIN_MAX_DELTA {
            // count low points on start side
            for i := 0; i <= N; i++ {
                if v[i] >= threshold {
                    break
                }
                num_start++
            }
            if num_start >= CAM_N_ACTIVE {
                // count low points on end side
                for i := CAM_N_POINTS - 1; i >= N; i-- {
                    if v[i] >= threshold {
                        break
                    }
                    num_end++
                }

                if num_end >= CAM_N_ACTIVE {
                    center = CAM_N_POINTS - num_start - num_end
                    if center >= CAM_N_ACTIVE {
                        is_empty = true
                    }
                }
            }
        }

        old := s.empty
        s.empty = is_empty
        if old != is_empty || s.init {
            m.SetSensor(s.sensor, !is_empty)
            s.init = false

            dispatchToSensorsServers(fmt.Sprintf("S:%d", s.sensor));

            if !is_empty {
                for _, t := range s.timers {
                    t.TimerTriggered(s.sensor)
                }
            }

            fmt.Printf("Cam Sensor changed: %v => %v [%d | %d | %d] %v\n",
                s.sensor, !is_empty, num_start, center, num_end, v)
        }
    }
}

func _abs(a, b int) int {
    if a < b {
        return b - a
    }
    return a - b
}

// Timer loop logic:
// - timer starts with start_time and ignore_time set to 0.
// - if start_time is zero, this is the first event. Set start time to now
//   and ignore to now + the ignore duration.
// - if start_time is not zero:
//   - if time > ignore_time, this is the end of the loop + start new one.
//   - if time <= ignore_time, just ignore it as a dummy event.
func (t *CamTimer) TimerTriggered(sensor int) {
    now := time.Now()

    if t.stop == nil {
        // Single loop timer
        if t.start.sensor == sensor {
            duration := t.start.SensorTriggered(now)
            if duration > 0 && duration.Hours() < 1 {
                t.mutex.Lock()
                defer t.mutex.Unlock()
                t.durations = append(t.durations, duration)
                fmt.Printf("Cam Timer [%v]: Loop %d = %v\n",
                    t.start.sensor, len(t.durations), duration)
                dispatchToSensorsServers(
                    fmt.Sprintf("L/%d:%f", t.start.sensor, duration.Seconds()))
            }
        }
    } else {
        // Start-stop dual timer
        if t.start.sensor == sensor {
            if t.start.SensorTriggered(now) > 0 {
                t.stop.trigger_time = t.start.trigger_time
            }
        } else if t.stop.sensor == sensor && t.stop.trigger_time == t.start.trigger_time {
            if t.stop.SensorTriggered(now) > 0 {
                duration := t.stop.trigger_time.Sub(t.start.trigger_time)
                if duration > 0 && duration.Hours() < 1 {
                    t.mutex.Lock()
                    defer t.mutex.Unlock()
                    t.durations = append(t.durations, duration)
                    fmt.Printf("Cam Timer [%v-%v]: %d = %v\n",
                        t.start.sensor, t.stop.sensor, len(t.durations), duration)
                    dispatchToSensorsServers(
                        fmt.Sprintf("T/%d-%d:%f", t.start.sensor, t.stop.sensor, duration.Seconds()))
                }
            }
        }
    }
}

func (s *CamTimerSensor) SensorTriggered(now time.Time) (duration time.Duration) {
    if !s.ignore_time.IsZero() && now.Before(s.ignore_time) {
        return duration
    }
    duration = now.Sub(s.trigger_time)
    s.trigger_time = now
    s.ignore_time = now.Add(CAM_TIMER_IGNORE)
    return duration
}



func runSensorsServer(m *Model) {
    fmt.Printf("Start Sensors server on %s\n", *SENSORS_SERV)

    listener, err := net.Listen("tcp", *SENSORS_SERV)
    if err != nil {
        panic(err)
    }

    go func() {
        for !m.IsQuitting() {
            conn, err := listener.Accept()
            if err != nil {
                panic(err)
            }

            go sensorsServerHandleConn(m, conn)
        }
    }()
}

func sensorsServerHandleConn(m *Model, conn net.Conn) {
    fmt.Printf("[Sensors server] Connection Open\n")

    c := CAM_SENSORS_CHANNELS.newChannel()

    defer func() {
        conn.Close()
        CAM_SENSORS_CHANNELS.delChannel(c)
        fmt.Printf("[Sensors server] Connection Closed\n")
    }()

    var err error
    for !m.IsQuitting() && err == nil {
        v := <-c
        _, err = conn.Write( []byte(v + "\n") )
    }
}

func (s *CamSensorsChannels) newChannel() chan string {
    s.mutex.Lock()
    defer s.mutex.Unlock()
    if s.channels == nil {
        s.channels = make(map[chan string]bool)
    }
    c := make(chan string, 100)
    s.channels[c] = true
    return c
}

func (s *CamSensorsChannels) delChannel(c chan string) {
    s.mutex.Lock()
    defer s.mutex.Unlock()
    delete(s.channels, c)
}

func dispatchToSensorsServers(data string) {
    CAM_SENSORS_CHANNELS.mutex.Lock()
    defer CAM_SENSORS_CHANNELS.mutex.Unlock()
    if CAM_SENSORS_CHANNELS.channels == nil {
        return
    }
    for c, ok := range CAM_SENSORS_CHANNELS.channels {
        if ok {
            go func(c chan string, s string) {
                c <- s
            }(c, data)
        }
    }
}
