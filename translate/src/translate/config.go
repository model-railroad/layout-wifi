package translate

import (
    "bufio"
    "fmt"
    "io"
    "os"
    "regexp"
)

type Config map[string] string

var CONFIG_LINE_RE *regexp.Regexp = regexp.MustCompile("^\\s*([a-zA-Z0-9_.-]+)\\s*=\\s*(.*?)\\s*$")

func NewConfig() Config {
    return Config{}
}

func (c Config) Read(r io.Reader) error {
    buf := bufio.NewReader(r)
    return c.parse(buf)
}

func (c Config) ReadFile(filename string) error {
    f, err := os.Open(filename)
    if err != nil {
        if os.IsNotExist(err) {
            fmt.Printf("[CONFIG] No config file %s found\n", filename)
            return nil
        } else {
            panic(fmt.Sprintf("[CONFIG] Error reading %v: %#v", filename, err))
        }
    }
    defer f.Close()
    return c.Read(f)
}

func (c Config) parse(r *bufio.Reader) error {
    for {
        line, err := r.ReadString('\n')
        if err == io.EOF {
            return nil
        } else if err != nil {
            panic(fmt.Sprintf("[CONFIG] Error reading config file: %v", err))
        }
        fields := CONFIG_LINE_RE.FindStringSubmatch(line)
        c[fields[1]] = fields[2]
    }
    return nil
}