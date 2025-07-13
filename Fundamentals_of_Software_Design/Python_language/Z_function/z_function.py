def compute_z_array(s):
    n = len(s)
    Z = [0] * n
    Z[0] = n
    l, r = 0, 0
    
    for i in range(1, n):
        if i > r:
            l = r = i
            while r < n and s[r - l] == s[r]:
                r += 1
            Z[i] = r - l
            r -= 1
        else:
            k = i - l
            if Z[k] < r - i + 1:
                Z[i] = Z[k]
            else:
                l = i
                while r < n and s[r - l] == s[r]:
                    r += 1
                Z[i] = r - l
                r -= 1
    return Z

def substring_search(text, pattern):
    if not pattern:
        return [i for i in range(len(text) + 1)]
    if not text:
        return []
    
    concat = pattern + '$' + text
    Z = compute_z_array(concat)
    result = []
    pattern_len = len(pattern)
    
    for i in range(len(Z)):
        if Z[i] == pattern_len:
            result.append(i - pattern_len - 1)
    
    return result
