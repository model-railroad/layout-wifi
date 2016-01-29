package translate

import (
    "testing"
    "time"
    "github.com/stretchr/testify/assert"
)

func TestQuitting(t *testing.T) {
    m := NewModel()

    assert.Equal(t, false, m.IsQuitting())

    m.SetQuitting()
    assert.Equal(t, true, m.IsQuitting())
}

func TestConvertSensorToAiuBit(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()

    aiu, bit := m.ConvertSensorToAiuBit(0+0)
    assert.Equal(1, aiu)
    assert.Equal(uint(0), bit)

    aiu, bit = m.ConvertSensorToAiuBit(0+13)
    assert.Equal(1, aiu)
    assert.Equal(uint(13), bit)

    aiu, bit = m.ConvertSensorToAiuBit(16+1)
    assert.Equal(2, aiu)
    assert.Equal(uint(1), bit)

    aiu, bit = m.ConvertSensorToAiuBit(32+12)
    assert.Equal(3, aiu)
    assert.Equal(uint(12), bit)

    aiu, bit = m.ConvertSensorToAiuBit(48+0)
    assert.Equal(4, aiu)
    assert.Equal(uint(0), bit)
}

func TestSensors(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()

    for i := 1; i <= MAX_AIUS; i++ {
        assert.Equal(uint16(0), m.GetSensors(i))

        m.SetSensors(i, uint16(i))
        assert.Equal(uint16(i), m.GetSensors(i))

        m.SetSensors(i, uint16(0))
        assert.Equal(uint16(0), m.GetSensors(i))
    }

    for i := 1; i <= MAX_AIUS; i++ {
        base := (i - 1) * 16
        for j := base; j < base + SENSORS_PER_AIU; j++ {

            assert.Equal(false, m.GetSensor(j))

            m.SetSensor(j, true)
            assert.Equal(true, m.GetSensor(j))
        }
    }

    for i := 1; i <= MAX_AIUS; i++ {
        assert.Equal(uint16(0x3FFF), m.GetSensors(i))
    }

    m.SetSensor(MAX_SENSORS - 1, false)
    m.SetSensor(MAX_SENSORS - 2, false)
    assert.Equal(uint16(0x0FFF), m.GetSensors(MAX_AIUS))
}

func TestTurnoutOp(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()

    op, ok := m.GetTurnoutOp(time.Millisecond)
    assert.Equal(false, ok)
    assert.Nil(op)

    m.SendTurnoutOp( &TurnoutOp{6, true} )
    
    op, ok = m.GetTurnoutOp(time.Millisecond)
    assert.Equal(true, ok)
    assert.Equal(TurnoutOp{6, true}, *op)    
}

func TestTurnoutStates(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()

    assert.Equal(uint32(0), m.GetTurnoutStates())
    assert.Equal(uint16(0), m.GetSensors(1))
    assert.Equal(uint16(0), m.GetSensors(2))
    
    m.SetTurnoutState( 1, true)
    m.SetTurnoutState( 2, false)
    m.SetTurnoutState( 5, true)
    m.SetTurnoutState( 6, false)
    m.SetTurnoutState( 9, false)
    m.SetTurnoutState(13, false)
    m.SetTurnoutState(17, false)
    m.SetTurnoutState(21, false)
    m.SetTurnoutState(25, false)
    m.SetTurnoutState(28, false)

    assert.Equal(uint32(0x9111122), m.GetTurnoutStates())
    assert.Equal(uint16(0x0000),    m.GetSensors(1))
    assert.Equal(uint16(0x0000),    m.GetSensors(2))
}
