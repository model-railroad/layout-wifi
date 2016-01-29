package translate

import (
    "fmt"
    "testing"
    "time"
    "github.com/stretchr/testify/assert"
)

func TestSrcpSession_ReplyRaw(t *testing.T) {
    assert := assert.New(t)
    c := newMockConn(nil)
    u := NewSrcpSessions()
    s := NewSrcpSession(42, SRCP_MODE_HANDSHAKE, 11, c, u)
    
    assert.Equal(42, s.id)
    assert.Equal(11, s.time)
    assert.Equal(SRCP_MODE_HANDSHAKE, s.mode)
    
    s.ReplyRaw("SRCP PROTOCOL")
    assert.Equal(11, s.time)
    assert.Equal( []byte("SRCP PROTOCOL\n"), c._write)    
}

func TestSrcpSession_Reply(t *testing.T) {
    assert := assert.New(t)
    c := newMockConn(nil)
    u := NewSrcpSessions()
    s := NewSrcpSession(42, SRCP_MODE_HANDSHAKE, 11, c, u)
    
    assert.Equal(42, s.id)
    assert.Equal(11, s.time)
    assert.Equal(SRCP_MODE_HANDSHAKE, s.mode)
    
    s.Reply("SRCP PROTOCOL")
    assert.Equal(12, s.time)
    assert.Equal( []byte("12 SRCP PROTOCOL\n"), c._write)    
}

// -----

func TestSrcpSessions_New(t *testing.T) {
    assert := assert.New(t)
    u := NewSrcpSessions()
    
    assert.NotNil(u)
    assert.Equal("", fmt.Sprintf("%v", u))
}

func TestSrcpSessions_Add(t *testing.T) {
    assert := assert.New(t)
    u := NewSrcpSessions()

    c := newMockConn( []byte{} )
    s := NewSrcpSession(42, SRCP_MODE_HANDSHAKE, 11, c, u)

    u.Add(s)    
    assert.Equal("42", fmt.Sprintf("%v", u))
}

func TestSrcpSessions_Remove(t *testing.T) {
    assert := assert.New(t)
    u := NewSrcpSessions()

    c := newMockConn( []byte{} )
    s := NewSrcpSession(42, SRCP_MODE_HANDSHAKE, 11, c, u)

    u.Add(s)    
    assert.Equal("42", fmt.Sprintf("%v", u))

    u.Remove(s)
    assert.Equal("", fmt.Sprintf("%v", u))
}

func TestSrcpSessions_Iter(t *testing.T) {
    assert := assert.New(t)
    u := NewSrcpSessions()

    c := newMockConn( []byte{} )
    s := NewSrcpSession(42, SRCP_MODE_HANDSHAKE, 11, c, u)

    u.Add(s)
    assert.Equal(42, func()int { 
        i := 0
        u.Iter( func(s1 *SrcpSession) {
            i += s1.id
        })
        return i
    }())
}


// -----

func _test_srcp_init(mode string, data []byte) (*Model, *MockConn, *SrcpSessions, *SrcpSession) {
    m := NewModel()
    c := newMockConn(data)
    u := NewSrcpSessions()
    s := NewSrcpSession(42, mode, 11, c, u)
    u.Add(s)
    return m, c, u, s
}

func TestSrcpSession_HandleConn_NoOp(t *testing.T) {
    m, c, _, s := _test_srcp_init(SRCP_MODE_HANDSHAKE, []byte{} )
    s.HandleConn(m)
    assert.Equal(t, []byte(SRCP_HEADER + "\n"), c._write)    
}

func TestSrcpSession_HandleLine_SetProto(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_HANDSHAKE, nil)
    s.HandleLine(m, "SET PROTOCOL SRCP blah")
    assert.Equal([]byte("12 201 OK PROTOCOL SRCP\n"), c._write)    
}

func TestSrcpSession_HandleLine_SetConnMode(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_HANDSHAKE, nil)
    s.HandleLine(m, "SET CONNECTIONMODE SRCP something")
    assert.Equal([]byte("12 202 OK CONNECTIONMODE\n"), c._write)    
    assert.Equal("something", s.mode)
}

func TestSrcpSession_HandleLine_Go_Command(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_HANDSHAKE, nil)
    s.HandleLine(m, "SET CONNECTIONMODE SRCP COMMAND")
    s.HandleLine(m, "GO")
    assert.Equal(
        []byte("12 202 OK CONNECTIONMODE\n" +
               "13 200 OK GO 42\n"), 
        c._write)    
}

func TestSrcpSession_HandleLine_Go_Info(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_HANDSHAKE, nil)
    m.SetSensor(MAX_SENSORS - 1, true)
    m.SetSensor(MAX_SENSORS - 2, true)
    s.HandleLine(m, "SET CONNECTIONMODE SRCP INFO")
    s.HandleLine(m, "GO")
    assert.Equal(
        []byte("12 202 OK CONNECTIONMODE\n" +
               "13 200 OK GO 42\n" +
               "14 100 INFO 0 DESCRIPTION SESSION SERVER TIME\n" +
               "15 100 INFO 0 TIME 1 1\n" +
               "16 100 INFO 0 TIME 12345 23 59 59\n" +
               "17 100 INFO 1 POWER ON controlled by digix\n" +
               "18 100 INFO 7 DESCRIPTION GA DESCRIPTION\n" +
               "19 100 INFO 8 DESCRIPTION FB DESCRIPTION\n" +
               // sensors set to zero are not transmitted
               "20 100 INFO 8 FB 60 1\n" +
               "21 100 INFO 8 FB 61 1\n"),
        c._write)    
}

func TestSrcpSession_HandleLine_TriggerTurnout(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_COMMAND, nil)
    s.HandleLine(m, "SET 7 GA 8 0")
    s.HandleLine(m, "SET 7 GA 8 1")

    assert.Equal(
        []byte("12 200 OK\n" +
               "13 200 OK\n"), 
        c._write)

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

func TestSrcpSession_SendSensorUpdate(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_INFO, nil)

    // No update at first -- both internal sensors and last send are at zero
    s.SendSensorUpdate(m)
    assert.Equal([]byte(nil), c._write)

    m.SetSensor(1, true)
    m.SetSensor(2, true)
    m.SetSensor(MAX_SENSORS - 2, true)
    m.SetSensor(MAX_SENSORS - 1, true)

    s.SendSensorUpdate(m)
    assert.Equal(
        "12 100 INFO 8 FB 1 1\n" +
        "13 100 INFO 8 FB 2 1\n" +
        "14 100 INFO 8 FB 60 1\n" +
        "15 100 INFO 8 FB 61 1\n",
        string(c._write))
    assert.Equal(
        []byte("12 100 INFO 8 FB 1 1\n" +
               "13 100 INFO 8 FB 2 1\n" +
               "14 100 INFO 8 FB 60 1\n" +
               "15 100 INFO 8 FB 61 1\n"),
        c._write)

    c.reset(nil)
    m.SetSensor(2, false)
    m.SetSensor(MAX_SENSORS - 2, false)
    s.SendSensorUpdate(m)
    assert.Equal(
        []byte("16 100 INFO 8 FB 2 0\n" +
               "17 100 INFO 8 FB 60 0\n"),
        c._write)

    c.reset(nil)
    m.SetSensor(1, false)
    m.SetSensor(MAX_SENSORS - 1, false)
    s.SendSensorUpdate(m)
    assert.Equal(
        []byte("18 100 INFO 8 FB 1 0\n" +
               "19 100 INFO 8 FB 61 0\n"),
        c._write)
}
