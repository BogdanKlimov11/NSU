def compute_lps_array(needle):
    lps = [0] * len(needle)
    length = 0
    i = 1
    
    while i < len(needle):
        if needle[i] == needle[length]:
            length += 1
            lps[i] = length
            i += 1
        else:
            if length != 0:
                length = lps[length - 1]
            else:
                lps[i] = 0
                i += 1
    return lps

def my_strstr(haystack, needle):
    if not needle:
        return 0
    if not haystack or len(needle) > len(haystack):
        return -1
    
    lps = compute_lps_array(needle)
    i = j = 0
    
    while i < len(haystack):
        if haystack[i] == needle[j]:
            i += 1
            j += 1
            if j == len(needle):
                return i - j
        else:
            if j != 0:
                j = lps[j - 1]
            else:
                i += 1
    return -1
