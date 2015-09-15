package translate

import (
    "testing"
    "github.com/stretchr/testify/assert"
)

// -----

func TestLwServ_New(t *testing.T) {
    assert := assert.New(t)
    s := NewLwServ()

    assert.NotNil(s)
}

func TestLwServ_HandleLine_NoOp(t *testing.T) {
    assert := assert.New(t)
    s := NewLwServ()

    assert.Equal("", s.HandleLine(""))
}

func TestLwServ_HandleLine_ReceiveInfo(t *testing.T) {
    assert := assert.New(t)
    s := NewLwServ()

    assert.Equal("@IT08S04\n", s.HandleLine("@I"))
}

func TestLwServ_HandleLine_TurnoutNormal(t *testing.T) {
    assert := assert.New(t)
    s := NewLwServ()

    assert.Equal(uint16(0x0000), s._sensors[0])
    assert.Equal("@T01N\n", s.HandleLine("@T01N"))
    assert.Equal(uint16(0x0000), s._sensors[0])
}

func TestLwServ_HandleLine_TurnoutReverse(t *testing.T) {
    assert := assert.New(t)
    s := NewLwServ()

    assert.Equal(uint16(0x0000), s._sensors[0])
    assert.Equal("@T01R\n", s.HandleLine("@T01R"))
    assert.Equal(uint16(0x0001), s._sensors[0])
}

func TestLwServ_PollSensors(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()
    s := NewLwServ()

    assert.Equal("@S010000\n@S020000\n@S030000\n@S040000\n", s.PollSensors(m))

    s.sensors_chan <- LwSensor{29,  true}
    s.sensors_chan <- LwSensor{42,  true}
    s.sensors_chan <- LwSensor{42, false}
    s.sensors_chan <- LwSensor{45,  true}

    assert.Equal("@S030001\n@S040004\n", s.PollSensors(m))
}
