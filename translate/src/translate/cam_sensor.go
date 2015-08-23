package translate

import (
    "flag"
    "fmt"
    "io"
    "image"
    "image/color"
    "image/jpeg"
    "mime/multipart"
    "regexp"
    "net/http"
    net_url "net/url"
    "strings"
    "strconv"
    "sync"
    "time"
)

const CAM_N_POINTS = 3

var CAM_SERV = flag.String("cam-server", ":8088", "Camera debug server host:port")

var CAM_HOSTS = flag.String("cam-urls",
    "camera1.local:80/path.cgi?user=foo&pwd=blah," +
    "user2:pwd2@camera2.local/path.cgi", 
    "Cameras user:pwd@host:port/path")

var CAM_SENSORS = flag.String("cam-sensors",
    "0,38:172,142,164,142,180,142 0,37:602,145,594,145,611,145",
    "Camera detection points: N,S:x,y,{3}...")
    
// Typical URLs:
// Foscam: http://foscam.ip/videostream.cgi?resolution=32&user=USER&pwd=PWD
// Edimax: http://USER:PWD@edimax.ip/mjpg/video.mjpg

//-----

var CAMERAS []*Camera

type CamSensor struct {
    sensor  int
    points  []int // CAM_N_POINTS pairs x/y
    values  []int // CAM_N_POINTS current values
    empty   bool
    init    bool
}

type Camera struct {
    index   int
    url     *net_url.URL
    mutex   sync.Mutex
    last    image.Image
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
type GrayImage struct {
    img   image.Image
    model color.Model
}

func (c *GrayImage) ColorModel() color.Model {
    return c.model
}

// Return the original bounds
func (c *GrayImage) Bounds() image.Rectangle {
    return c.img.Bounds()
}

// At() forwards the call to the original image and
// uses the color model to convert it.
func (c *GrayImage) At(x, y int) color.Color {
    return c.model.Convert(c.img.At(x,y))
}

//-----

func CamSensorDebugServer() {
    http.HandleFunc("/", CamSensorDebugHandler)

    if err := http.ListenAndServe(*CAM_SERV, nil /*handler*/); err != nil {
        panic(err)
    }
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
        w.Header().Set("Content-Type", "image/jpeg")
        w.WriteHeader(http.StatusOK)
        jpeg.Encode(w, img, nil /*Options*/)
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
            panic(err)
        }
        
        //-- bytes, err := ioutil.ReadAll(part)
        defer part.Close()
        
        img, err := jpeg.Decode(part)
        if err != nil {
            panic(err)
        }
        
        if img.ColorModel() != color.GrayModel {
            img = &GrayImage{ img, color.GrayModel }
        }
                
        cam.UpdateSensors(img, m)
        cam.SetLast(img)
        
        /* fmt.Printf("[CAM %d: %s] Decoded JPEG %d x %d\n", 
            cam.index, cam.url.Host, 
            img.Bounds().Dx(), img.Bounds().Dy())
        */
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

func (cam *Camera) SetupSensors(config string) {
    re := regexp.MustCompile(fmt.Sprintf("([%d]),([0-9]+):([0-9]+),([0-9]+),([0-9]+),([0-9]+),([0-9]+),([0-9]+)", cam.index))
    
    match := re.FindAllStringSubmatch(config, -1 /*all*/)
    for _, result := range match {
        s := &CamSensor{}
        s.init = true
        
        var err error
        if s.sensor, err = strconv.Atoi(result[2]); err != nil {
            panic(err)
        }
        
        s.points = make([]int, CAM_N_POINTS * 2)
        s.values = make([]int, CAM_N_POINTS)
        for i := 0; i < CAM_N_POINTS * 2; i++ {
            if s.points[i], err = strconv.Atoi(result[3 + i]); err != nil {
                panic(err)
            }
        }
        
        cam.sensors = append(cam.sensors, s)
    }
}

func (cam *Camera) DebugHtml() string {
    content := fmt.Sprintf(
        `<p/><a href='/last/%d'>Last image from camera %d</a><br/>
         <img src='/last/%d'/><br/>`, 
        cam.index, cam.index, cam.index)
        
    for index, sensor := range cam.sensors {
        content += fmt.Sprintf("[%d] %v <br/>", index, sensor)
    }
        
    return content
}

func (cam *Camera) UpdateSensors(img image.Image, m *Model) {
    for _, sensor := range cam.sensors {
        var i, j int
        for i < CAM_N_POINTS{
            c := img.At(sensor.points[j], sensor.points[j+1])
            sensor.values[i] = int(c.(color.Gray).Y)
            i++
            j += 2 
            
            // value 0 is the mid point, which must be high (white)
            // values 1 & 2 are the guards (darker).
            // Expected: [0] > 128 && [1,2] < 128
            v := sensor.values
            old := sensor.empty
            is_empty := v[0] > 128 && v[1] < 128 && v[2] < 128
            sensor.empty = is_empty
            if old != is_empty || sensor.init {
                m.SetSensor(sensor.sensor, !is_empty)
                sensor.init = false
            }
        }
    }
}


