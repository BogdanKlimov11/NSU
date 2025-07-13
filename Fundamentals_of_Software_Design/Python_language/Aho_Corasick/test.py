import unittest
from aho_corasick import substring_search
from random import choice, randint
import string

class TestAhoCorasick(unittest.TestCase):
    def test_empty_pattern(self):
        self.assertEqual(substring_search("abc", ""), [0, 1, 2, 3])
    
    def test_empty_text(self):
        self.assertEqual(substring_search("", "a"), [])
    
    def test_single_match(self):
        self.assertEqual(substring_search("abcde", "bcd"), [1])
    
    def test_multiple_matches(self):
        self.assertEqual(substring_search("ababab", "aba"), [0, 2])
    
    def test_no_match(self):
        self.assertEqual(substring_search("abcdef", "xyz"), [])
    
    def test_pattern_equals_text(self):
        self.assertEqual(substring_search("abc", "abc"), [0])
    
    def test_overlapping(self):
        self.assertEqual(substring_search("aaa", "aa"), [0, 1])
    
    def test_unicode(self):
        self.assertEqual(substring_search("привет мир", "мир"), [7])
    
    def test_large_text(self):
        text = "a" * 1000000
        self.assertEqual(substring_search(text, "a" * 100), list(range(1000000 - 99)))
    
    def test_random_patterns(self):
        text = ''.join(choice(string.ascii_lowercase) for _ in range(10000))
        pattern = text[randint(0, 9000):randint(1000, 10000)]
        positions = [i for i in range(len(text)) if text.startswith(pattern, i)]
        self.assertEqual(substring_search(text, pattern), positions)

if __name__ == "__main__":
    unittest.main()
