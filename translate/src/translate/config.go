package translate

import (
    "bufio"
    "flag"
    "fmt"
    "io"
    "os"
    "regexp"
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
            return nil
        } else if err != nil {
            panic(fmt.Sprintf("[CONFIG] Error reading config file: %v", err))
        }
        fields := CONFIG_LINE_RE.FindStringSubmatch(line)
        (*c)[fields[1]] = fields[2]
    }
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

func (c *Config) UpdateFlags(flags *flag.FlagSet) {
    var actual = make(map[string] *flag.Flag) 
    flag.Visit(func(f *flag.Flag) {
        actual[f.Name] = f
    })
    
    flag.VisitAll(func(f *flag.Flag) {
        if _, visited := actual[f.Name]; !visited {
            if value, ok := (*c)[f.Name]; ok {
                flags.Set(f.Name, value)
            }
        }
    })
}
