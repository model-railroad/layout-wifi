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

func TestHandleSrcpConn_NoOp(t *testing.T) {
    m, c, _, s := _test_srcp_init(SRCP_MODE_HANDSHAKE, []byte{} )
    HandleSrcpConn(m, s)
    assert.Equal(t, []byte(SRCP_HEADER + "\n"), c._write)    
}

func TestHandleSrcpLine_SetProto(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_HANDSHAKE, nil)
    HandleSrcpLine(m, s, "SET PROTOCOL SRCP blah")
    assert.Equal([]byte("12 201 OK PROTOCOL SRCP\n"), c._write)    
}

func TestHandleSrcpLine_SetConnMode(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_HANDSHAKE, nil)
    HandleSrcpLine(m, s, "SET CONNECTIONMODE SRCP something")
    assert.Equal([]byte("12 202 OK CONNECTIONMODE\n"), c._write)    
    assert.Equal("something", s.mode)
}

func TestHandleSrcpLine_Go_Command(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_HANDSHAKE, nil)
    HandleSrcpLine(m, s, "SET CONNECTIONMODE SRCP COMMAND")
    HandleSrcpLine(m, s, "GO")
    assert.Equal(
        []byte("12 202 OK CONNECTIONMODE\n" +
               "13 200 OK GO 42\n"), 
        c._write)    
}

func TestHandleSrcpLine_Go_Info(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_HANDSHAKE, nil)
    m.SetSensor(MAX_SENSORS, true)
    m.SetSensor(MAX_SENSORS - 1, true)
    HandleSrcpLine(m, s, "SET CONNECTIONMODE SRCP INFO")
    HandleSrcpLine(m, s, "GO")
    assert.Equal(
        []byte("12 202 OK CONNECTIONMODE\n" +
               "13 200 OK GO 42\n" +
               "14 100 INFO 0 DESCRIPTION SESSION SERVER TIME\n" +
               "15 100 INFO 0 TIME 1 1\n" +
               "16 100 INFO 0 TIME 12345 23 59 59\n" +
               "17 100 INFO 1 POWER ON controlled by digix\n" +
               "18 100 INFO 7 DESCRIPTION GA DESCRIPTION\n" +
 /*            "19 100 INFO 7 GA 1 0 1\n" +
               "20 100 INFO 7 GA 1 1 0\n" +
               "21 100 INFO 7 GA 2 0 1\n" +
               "22 100 INFO 7 GA 2 1 0\n" +
               "23 100 INFO 7 GA 3 0 1\n" +
               "24 100 INFO 7 GA 3 1 0\n" +
               "25 100 INFO 7 GA 4 0 1\n" +
               "26 100 INFO 7 GA 4 1 0\n" +
               "27 100 INFO 7 GA 5 0 1\n" +
               "28 100 INFO 7 GA 5 1 0\n" +
               "29 100 INFO 7 GA 6 0 1\n" +
               "30 100 INFO 7 GA 6 1 0\n" +
               "31 100 INFO 7 GA 7 0 1\n" +
               "32 100 INFO 7 GA 7 1 0\n" +
               "33 100 INFO 7 GA 8 0 1\n" +
               "34 100 INFO 7 GA 8 1 0\n" +
               "35 100 INFO 7 GA 9 0 1\n" +
               "36 100 INFO 7 GA 9 1 0\n" +
               "37 100 INFO 7 GA 10 0 1\n" +
               "38 100 INFO 7 GA 10 1 0\n" +
               "39 100 INFO 7 GA 11 0 1\n" +
               "40 100 INFO 7 GA 11 1 0\n" +
               "41 100 INFO 7 GA 12 0 1\n" +
               "42 100 INFO 7 GA 12 1 0\n" +
               "43 100 INFO 7 GA 13 0 1\n" +
               "44 100 INFO 7 GA 13 1 0\n" +
               "45 100 INFO 7 GA 14 0 1\n" +
               "46 100 INFO 7 GA 14 1 0\n" +
               "47 100 INFO 7 GA 15 0 1\n" +
               "48 100 INFO 7 GA 15 1 0\n" +
               "49 100 INFO 7 GA 16 0 1\n" +
               "50 100 INFO 7 GA 16 1 0\n" +
               "51 100 INFO 7 GA 17 0 1\n" +
               "52 100 INFO 7 GA 17 1 0\n" +
               "53 100 INFO 7 GA 18 0 1\n" +
               "54 100 INFO 7 GA 18 1 0\n" +
               "55 100 INFO 7 GA 19 0 1\n" +
               "56 100 INFO 7 GA 19 1 0\n" +
               "57 100 INFO 7 GA 20 0 1\n" +
               "58 100 INFO 7 GA 20 1 0\n" +
               "59 100 INFO 7 GA 21 0 1\n" +
               "60 100 INFO 7 GA 21 1 0\n" +
               "61 100 INFO 7 GA 22 0 1\n" +
               "62 100 INFO 7 GA 22 1 0\n" +
               "63 100 INFO 7 GA 23 0 1\n" +
               "64 100 INFO 7 GA 23 1 0\n" +
               "65 100 INFO 7 GA 24 0 1\n" +
               "66 100 INFO 7 GA 24 1 0\n" +
               "67 100 INFO 7 GA 25 0 1\n" +
               "68 100 INFO 7 GA 25 1 0\n" +
               "69 100 INFO 7 GA 26 0 1\n" +
               "70 100 INFO 7 GA 26 1 0\n" +
               "71 100 INFO 7 GA 27 0 1\n" +
               "72 100 INFO 7 GA 27 1 0\n" +
               "73 100 INFO 7 GA 28 0 1\n" +
               "74 100 INFO 7 GA 28 1 0\n" + */
               "19 100 INFO 8 DESCRIPTION FB DESCRIPTION\n" +
               // sensors set to zero are not transmitted
               "20 100 INFO 8 FB 55 1\n" +
               "21 100 INFO 8 FB 56 1\n"), 
        c._write)    
}

func TestHandleSrcpLine_TriggerTurnout(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_COMMAND, nil)
    HandleSrcpLine(m, s, "SET 7 GA 8 0")
    HandleSrcpLine(m, s, "SET 7 GA 8 1")

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

func TestSendSrcpSensorUpdate(t *testing.T) {
    assert := assert.New(t)
    m, c, _, s := _test_srcp_init(SRCP_MODE_INFO, nil)

    // No update at first -- both internal sensors and last send are at zero
    SendSrcpSensorUpdate(m, s)
    assert.Equal([]byte(nil), c._write)

    m.SetSensor(1, true)
    m.SetSensor(2, true)
    m.SetSensor(MAX_SENSORS - 1, true)
    m.SetSensor(MAX_SENSORS, true)

    SendSrcpSensorUpdate(m, s)
    assert.Equal(
        []byte("12 100 INFO 8 FB 1 1\n" +
               "13 100 INFO 8 FB 2 1\n" +
               "14 100 INFO 8 FB 55 1\n" +
               "15 100 INFO 8 FB 56 1\n"), 
        c._write)

    c.reset(nil)
    m.SetSensor(2, false)
    m.SetSensor(MAX_SENSORS - 1, false)
    SendSrcpSensorUpdate(m, s)
    assert.Equal(
        []byte("16 100 INFO 8 FB 2 0\n" +
               "17 100 INFO 8 FB 55 0\n"), 
        c._write)

    c.reset(nil)
    m.SetSensor(1, false)
    m.SetSensor(MAX_SENSORS, false)
    SendSrcpSensorUpdate(m, s)
    assert.Equal(
        []byte("18 100 INFO 8 FB 1 0\n" +
               "19 100 INFO 8 FB 56 0\n"), 
        c._write)
}
