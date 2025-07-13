def bad_char_table(pattern):
    table = {}
    length = len(pattern)
    for i in range(length - 1):
        table[pattern[i]] = length - i - 1
    return table

def my_strstr(haystack, needle):
    if not needle:
        return 0
    if not haystack or len(needle) > len(haystack):
        return -1

    bc_table = bad_char_table(needle)
    n_len = len(needle)
    h_len = len(haystack)
    skip = 0

    while h_len - skip >= n_len:
        i = n_len - 1
        while haystack[skip + i] == needle[i]:
            if i == 0:
                return skip
            i -= 1
        skip += bc_table.get(haystack[skip + n_len - 1], n_len)
    return -1
