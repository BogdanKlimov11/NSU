def polynomial_hash(s, base=256, modulus=101):
    hash_value = 0
    for char in s:
        hash_value = (hash_value * base + ord(char)) % modulus
    return hash_value

def update_hash(prev_hash, prev_char, new_char, length, base=256, modulus=101):
    prev_hash = (prev_hash - ord(prev_char) * pow(base, length-1, modulus)) % modulus
    return (prev_hash * base + ord(new_char)) % modulus

def my_strstr(haystack, needle):
    if not needle:
        return 0
    if not haystack or len(needle) > len(haystack):
        return -1
    
    n = len(needle)
    m = len(haystack)
    base = 256
    modulus = 101
    
    needle_hash = polynomial_hash(needle, base, modulus)
    window_hash = polynomial_hash(haystack[:n], base, modulus)
    
    if needle_hash == window_hash and haystack[:n] == needle:
        return 0
    
    for i in range(1, m - n + 1):
        window_hash = update_hash(
            window_hash,
            haystack[i-1],
            haystack[i+n-1],
            n,
            base,
            modulus
        )
        
        if needle_hash == window_hash and haystack[i:i+n] == needle:
            return i
    
    return -1
