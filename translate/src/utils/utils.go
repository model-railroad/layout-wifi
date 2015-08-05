// Copyright ralf

package utils

import (
    "fmt"
    "strings"
)

func MinInt(a, b int) int {
    if a < b {
        return a
    }
    return b
}

func MaxInt(a, b int) int {
    if a > b {
        return a
    }
    return b
}

func DiffBytes(b1, b2 []byte) string {
    return DiffStrings(string(b1), string(b2))
}

func DiffStrings(s1, s2 string) string {
    var result = ""
    a1 := strings.Split(s1, "\n")
    l1 := len(a1)
    a2 := strings.Split(s2, "\n")
    l2 := len(a2)
    
    maxLen := MaxInt(l1, l2)
    for i := 0; i < maxLen; i++ {
        if s1 = "" ; i < l1 { s1 = fmt.Sprintf("%q", a1[i]) }
        if s2 = "" ; i < l2 { s2 = fmt.Sprintf("%q", a2[i]) }
        if s1 == s2 {
            result = result + "\n    " + s1
        } else if s1 < s2 {
            result = result + "\n<-- " + s1 + "\n... " + s2
        } else if s1 > s2 {
            result = result + "\n--> " + s1 + "\n... " + s2
        }
    }
    
    return result
}
