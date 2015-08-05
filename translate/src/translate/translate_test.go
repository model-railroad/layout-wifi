package translate

import (
    "testing"
    "github.com/stretchr/testify/assert"
)

func TestNewModel(t *testing.T) {
    m := NewModel()
    assert.NotNil(t, m)
    assert.Equal(t, false, m.IsQuitting())
}
