package translate

import (
    "strings"
    "testing"
    "github.com/stretchr/testify/assert"
)

func TestConfig_Empty(t *testing.T) {
    assert := assert.New(t)

    r := strings.NewReader("")
    
    c := NewConfig()
    assert.Nil(c.Read(r))
    assert.Equal(0, len(*c))    
}

func TestConfig_Content(t *testing.T) {
    assert := assert.New(t)

    r := strings.NewReader("key1=value1\n" +
                            "    key_2 = \t some other value     \n" +
                            "    __KEY-3__   = value for 3  \t   \n")
    
    c := NewConfig()
    assert.Nil(c.Read(r))
    assert.Equal(3, len(*c))
    assert.Equal("value1", (*c)["key1"])
    assert.Equal("some other value", (*c)["key_2"])
    assert.Equal("value for 3", (*c)["__KEY-3__"])
    
    assert.Equal("value1", c.Get("key1", "default"))
    assert.Equal("some other value", c.Get("key_2", "default"))
    assert.Equal("value for 3", c.Get("__KEY-3__", "default"))

    assert.Equal("default", c.Get("key4", "default"))
}
