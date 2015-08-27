package translate

import (
    "bufio"
    "flag"
    "fmt"
    "io"
    "os"
    "regexp"
    "strings"
)

type Config map[string] string

var CONFIG_LINE_RE *regexp.Regexp = regexp.MustCompile("^\\s*([a-zA-Z0-9_.-]+)\\s*=\\s*(.*?)\\s*$")

func NewConfig() *Config {
    return &Config{}
}

func (c *Config) Read(r io.Reader) error {
    buf := bufio.NewReader(r)
    return c.parse(buf)
}

func (c *Config) ReadFile(filename string) error {
    fmt.Printf("[CONFIG] filename: %s\n", filename)
    f, err := os.Open(filename)
    if err != nil {
        if os.IsNotExist(err) {
            fmt.Printf("[CONFIG] %s not found\n", filename)
            return nil
        } else {
            panic(fmt.Sprintf("[CONFIG] Error reading %v: %#v", filename, err))
        }
    }
    defer f.Close()
    return c.Read(f)
}

func (c *Config) parse(r *bufio.Reader) error {
    for {
        line, err := r.ReadString('\n')
        if err == io.EOF {
            break
        } else if err != nil {
            panic(fmt.Sprintf("[CONFIG] Error reading config file: %v", err))
        }
        line = strings.TrimSpace(line)
        if line != "" {
            fields := CONFIG_LINE_RE.FindStringSubmatch(line)
            (*c)[fields[1]] = fields[2]
        }
    }
    fmt.Printf("[CONFIG] Read %d key/values from config file\n", len(*c))
    return nil
}

func (c *Config) Get(key, defaultValue string) string {
    value, ok := (*c)[key]
    if ok {
        return value
    } else {
        return defaultValue
    }
}

func (c *Config) UpdateFlags(fs *flag.FlagSet) {
    var actual = make(map[string] *flag.Flag) 
    fs.Visit(func(f *flag.Flag) {
        actual[f.Name] = f
    })
    
    fs.VisitAll(func(f *flag.Flag) {
        if _, visited := actual[f.Name]; !visited {
            if value, ok := (*c)[f.Name]; ok {
                fs.Set(f.Name, value)
                //--fmt.Printf("[CONFIG] DEBUG: SET %s = %s\n", f.Name, value)
            }
        }
    })
}
