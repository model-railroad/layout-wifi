package translate

import (
    "flag"
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

func TestConfig_Flags(t *testing.T) {
    assert := assert.New(t)

    fs := flag.NewFlagSet("test_flags", flag.PanicOnError)
    i1 := fs.Int   ("int-1",  -1 , "usage 1")
    s2 := fs.String("str-2", "-2", "usage 2")
    i3 := fs.Int   ("int-3",  -3 , "usage 3")
    s4 := fs.String("str-4", "-4", "usage 4")
    assert.Nil(fs.Parse( []string { "--int-1", "1", "--str-4", "4" } ))

    assert.Equal(  1 , *i1) // using parsed value
    assert.Equal("-2", *s2)
    assert.Equal( -3 , *i3)
    assert.Equal( "4", *s4) // using parsed value
    
    r := strings.NewReader("int-1 = 42\n" +
                           "str-2 = value 2\n" +
                           "int-3 = 43\n")
    c := NewConfig()
    assert.Nil(c.Read(r))

    c.UpdateFlags(fs)
    assert.Equal(  1 , *i1) // not overriding parsed value
    assert.Equal("value 2", *s2)
    assert.Equal( 43 , *i3)
    assert.Equal( "4", *s4)
}