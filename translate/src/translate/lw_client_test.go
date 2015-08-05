package translate

import (
    "testing"
    "github.com/stretchr/testify/assert"
)

// -----


func TestHandleLwClientReadLine_Info(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()

    assert.Nil(HandleLwClientReadLine(m, "@IT04S01"))
}

func TestHandleLwClientReadLine_Turnout(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()
    assert.Equal(uint32(0), m.GetTurnoutStates())

    assert.Nil(HandleLwClientReadLine(m, "@T01N"))
    assert.Equal(uint32(0), m.GetTurnoutStates())

    assert.Nil(HandleLwClientReadLine(m, "@T01R"))
    assert.Equal(uint32(0x0001), m.GetTurnoutStates())
}

func TestHandleLwClientReadLine_Sensors(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()

    assert.Nil(HandleLwClientReadLine(m, "@S030000"))
    assert.Equal(uint16(0), m.GetSensors(AIU_SENSORS_BASE))

    assert.Nil(HandleLwClientReadLine(m, "@S032001"))
    assert.Equal(uint16(0x2001), m.GetSensors(AIU_SENSORS_BASE))
}
