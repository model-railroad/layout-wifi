package translate

import (
    "fmt"
    "sync"
    "sync/atomic"
    "time"
)

const CONTINUE = 0
const QUITTING = 1

// The 2 first AIUs are to simulate turnout feedback.
// Expect to have a dozen block detection sensors, so
// let's give 1 extra for future expansion.
const MAX_AIUS        = 4
// The first AIU that contains LayoutWifi sensors is internally 3
const AIU_SENSORS_BASE= 3
const SENSORS_PER_AIU = 14
const SENSORS_MASK    = (1 << SENSORS_PER_AIU) - 1
const MAX_SENSORS     = SENSORS_PER_AIU * MAX_AIUS

// Expect to handle 8 turnouts, using 28 for future expansion.
// By convention AIU 1 and 2 will mirror the turnout feedback states.
// Actual block detection sensors will start at AIU 3
const MAX_TURNOUTS = 2 * SENSORS_PER_AIU

type TurnoutOp struct {
    // Turnout Addresses are 1-based: 1..MAX_TURNOUTS
    Address int
    Normal  bool
}

type Model struct {
    quitting int32  // Atomic access only

    turnoutOps chan *TurnoutOp
    
    // All fields below access only when the Mutex is held
    mutex   sync.Mutex

    // One uint16 per AIU, which carry 14-bits each.
    sensors []uint16
    
    // Turnout state, one bit per turnout.
    // Bit set to 0 means normal, set to 1 means reverse.
    // Expect to handle 8 turnouts, using 32-bits for future expansion.
    turnouts uint32
}

func NewModel() *Model {
    m := &Model{}
    m.quitting = CONTINUE
    m.turnoutOps = make(chan *TurnoutOp, 16)
    
    m.mutex.Lock()
    defer m.mutex.Unlock()
    
    m.sensors = make([]uint16, MAX_AIUS)
    return m
}

func (m *Model) SetQuitting() {
    atomic.StoreInt32(&m.quitting, QUITTING)
}

func (m *Model) IsQuitting() bool {
    return atomic.LoadInt32(&m.quitting) == QUITTING
}

// Get the 1-bit sensor value for the given sensor.
// Sensors numbers are 1-based: 1..MAX_SENSORS
func (m *Model) GetSensor(sensor int) bool {
    if (sensor < 1 || sensor > MAX_SENSORS) {
        panic(fmt.Errorf("Invalid sensor number %d [1..%d]", sensor, MAX_SENSORS))
    }

    m.mutex.Lock()
    defer m.mutex.Unlock()

    sensor -= 1
    aiu   := sensor / SENSORS_PER_AIU
    index := uint(sensor - aiu * SENSORS_PER_AIU)

    return ((m.sensors[aiu] >> index) & 1) != 0
}

// Get the 14-bit sensor value for the given AIU.
// AIU numbers are 1-based: 1..MAX_AIUS
func (m *Model) GetSensors(aiu int) uint16 {
    m.mutex.Lock()
    defer m.mutex.Unlock()

    if (aiu < 1 || aiu > MAX_AIUS) {
        panic(fmt.Errorf("Invalid AIU number %d [1..%d]", aiu, MAX_AIUS))
    }

    return m.sensors[aiu - 1]
}

// Set the 1-bit sensor value for the given sensor.
// Sensors numbers are 1-based: 1..MAX_SENSORS
func (m *Model) SetSensor(sensor int, active bool) {
    if (sensor < 1 || sensor > MAX_SENSORS) {
        panic(fmt.Errorf("Invalid sensor number %d [1..%d]", sensor, MAX_SENSORS))
    }

    m.mutex.Lock()
    defer m.mutex.Unlock()

    sensor -= 1
    aiu   := sensor / SENSORS_PER_AIU
    index := uint(sensor - aiu * SENSORS_PER_AIU)

    n := m.sensors[aiu]
    if active {
        n |= 1 << index
    } else {
        n &= ^(1 << index)
    }
    m.sensors[aiu] = n
}

// Set the 14-bit sensors value for the given AIU.
// AIU numbers are 1-based: 1..MAX_AIUS
func (m *Model) SetSensors(aiu int, sensors uint16) {
    m.mutex.Lock()
    defer m.mutex.Unlock()

    if (aiu < 1 || aiu > MAX_AIUS) {
        panic(fmt.Errorf("Invalid AIU number %d [1..%d]", aiu, MAX_AIUS))
    }

    m.sensors[aiu - 1] = sensors & SENSORS_MASK
}

func (m *Model) SendTurnoutOp(op *TurnoutOp) {
    if (op.Address < 1 || op.Address > MAX_TURNOUTS) {
        panic(fmt.Errorf("Invalid turnout number %d [1..%d]", op.Address, MAX_TURNOUTS))
    }

    m.turnoutOps <- op
}

// Blocks for the duration and returns the next turnout op available or nil
func (m *Model) GetTurnoutOp(d time.Duration) (op *TurnoutOp, ok bool) {
    select {
    case op = <- m.turnoutOps:
        ok = true
    case <- time.After(d):
        op = nil
        ok = false
    }
    return op, ok
}

func (m *Model) SetTurnoutState(index uint, normal bool) {
    if (index < 1 || index > MAX_TURNOUTS) {
        panic(fmt.Errorf("Invalid turnout index %d [1..%d]", index, MAX_TURNOUTS))
    }
    index -= 1

    m.mutex.Lock()
    defer m.mutex.Unlock()

    n := m.turnouts
    if normal {
        n &= ^(1 << index)
    } else {
        n |= 1 << index
    }
    m.turnouts = n
}

func (m *Model) GetTurnoutStates() uint32 {
    m.mutex.Lock()
    defer m.mutex.Unlock()

    return m.turnouts
}
