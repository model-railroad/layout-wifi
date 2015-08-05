package translate

import (
    "io"
    "net"
    "testing"
    "time"
    "utils"
    "github.com/stretchr/testify/assert"
)

// ----
// Type MockConn implements the net.Conn interface

type MockConn struct {
    _read   []byte
    _write  []byte 
    _closed bool 
}

func newMockConn(_read []byte) *MockConn {
    c := &MockConn{}
    c.reset(_read)
    return c
}

func (c *MockConn) reset(_read []byte) {
    c._read = _read
    c._write = nil
    c._closed = false
}

func (c *MockConn) Close() error {
    c._closed = true
    return nil
}

func (c *MockConn) Read(b []byte) (n int, err error) {
    n = utils.MinInt(len(b), len(c._read))
    if n > 0 {
        copy(b, c._read[0:n])
        c._read = c._read[n:]
        return n, nil

    } else {    
        return n, &net.OpError{"EOF", "tcp", nil, io.EOF}
    }
}

func (c *MockConn) Write(b []byte) (n int, err error) {
    if c._write == nil {
        c._write = make([]byte, 0)
    }

    // Note: "b..." means to append a slice to a slice, otherwise it appends a single byte
    c._write = append(c._write, b...)
    return 0, nil
}

func (c *MockConn) LocalAddr() net.Addr {
    return nil
}

func (c *MockConn) RemoteAddr() net.Addr {
    return nil
}

func (c *MockConn) SetDeadline(t time.Time) error {
    return nil
}

func (c *MockConn) SetReadDeadline(t time.Time) error {
    return nil
}

func (c *MockConn) SetWriteDeadline(t time.Time) error {
    return nil
}

// ---

func TestMockConn_Read(t *testing.T) {
    assert := assert.New(t)
    c := newMockConn( []byte{ 1, 2, 3, 4, 5 } )

    buf2 := make([]byte, 2)
    buf3 := make([]byte, 3)
    
    n, err := c.Read(buf2)
    assert.Equal(2, n)
    assert.Nil(err)
    
    n, err = c.Read(buf3)
    assert.Equal(3, n)
    assert.Nil(err)
    
    n, err = c.Read(buf2)
    assert.Equal(0, n)
    assert.NotNil(t, err)
}

func TestMockConn_Write(t *testing.T) {
    assert := assert.New(t)
    c := newMockConn( []byte{ 1, 2, 3, 4, 5 } )
    assert.Nil(c._write)

    c.Write( []byte{ 6, 7, 8 } )
    assert.Equal([]byte{ 6, 7, 8 }, c._write)

    c.Write( []byte{ 9 } )
    assert.Equal([]byte{ 6, 7, 8, 9 }, c._write)
}

func TestMockConn_Close(t *testing.T) {
    assert := assert.New(t)
    c := newMockConn( []byte{ 1, 2, 3, 4, 5 } )
    assert.Equal(false, c._closed)

    c.Close()
    assert.Equal(true, c._closed)
}

// ---

func TestHandleNceConn_Version(t *testing.T) {
    m := NewModel()
    c := newMockConn( []byte{ NCE_GET_VERSION } )
    HandleNceConn(m, c)
    // version 6.3.8
    assert.Equal(t, []byte{ 6, 3, 8 }, c._write)
}

func TestHandleNceConn_GetSensors(t *testing.T) {
    m := NewModel()
    m.SetSensors(2, 0x1234)

    c := newMockConn( []byte{ NCE_GET_AIU_SENSORS, 1,  
                              NCE_GET_AIU_SENSORS, 2,  
                              NCE_GET_AIU_SENSORS, 2 } )
    HandleNceConn(m, c)
    assert.Equal(t, []byte{ 0x00, 0x00, 0x00, 0x00,
                            0x12, 0x34, 0x12, 0x34,
                            0x12, 0x34, 0x00, 0x00 }, c._write)
}

func TestHandleNceConn_TriggerTurnout(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()

    c := newMockConn( []byte{ NCE_TRIGGER_ACC, 0x00, 0x08, 3, 0, 
                              NCE_TRIGGER_ACC, 0x00, 0x08, 4, 0 } )
    HandleNceConn(m, c)
    assert.Equal([]byte{ '!', '!' }, c._write)

    op, ok := m.GetTurnoutOp(time.Millisecond)
    assert.Equal(true, ok)
    assert.Equal(TurnoutOp{0x08, true}, *op)

    op, ok = m.GetTurnoutOp(time.Millisecond)
    assert.Equal(true, ok)
    assert.Equal(TurnoutOp{0x08, false}, *op)
    
    op, ok = m.GetTurnoutOp(time.Millisecond)
    assert.Equal(false, ok)
    assert.Nil(op)
}

func TestHandleNceConn_ReadRAM(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()

    c := newMockConn( []byte{ NCE_READ_RAM, 0x01, 0x23 } )
    HandleNceConn(m, c)
    assert.Equal([]byte{ 0 }, c._write)
}

func TestHandleNceConn_ReadTurnouts(t *testing.T) {
    assert := assert.New(t)
    m := NewModel()

    c := newMockConn( []byte{ NCE_READ_TURNOUTS, 0x01, 0x23 } )
    HandleNceConn(m, c)
    assert.Equal([]byte{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
                 c._write)

    m.SetTurnoutState(1, true)
    m.SetTurnoutState(2, false)
    m.SetTurnoutState(5, true)
    m.SetTurnoutState(6, false)
                 
    c.reset( []byte{ NCE_READ_TURNOUTS, 0x01, 0x23 } )
    HandleNceConn(m, c)
    assert.Equal([]byte{ 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
                 c._write)
}
