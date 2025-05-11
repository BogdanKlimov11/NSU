#include <vector>
#include <iostream>

#include "string.hpp"

using namespace std;

int main() {
    String str2("abcd");
    String str3("abcdefg", 4);
    String str4(3, 'a');
    String str5(str2);
    String str6(str3, 2, 2);
    cout << str2.data() << " " << str3.data() << " " << 
        str4.data() << " " << str5.data() << " " << str6.data() << " " << endl;
    cout << str2.size() << " " << str2.capacity() << " " <<
        str2.at(2) << " " << str2[2] << " " << str2.back() << " " <<
        str2.front() << " " << str2.find('b') << " " <<
        str2.compare("abcd") << " " << str2.countRef() << " " <<
        str2.empty() << " " << str2.substr(1, 2).data() << endl;
    str2.clear();
    str2.insert(0, "meMememe");
    cout << str2.data() << " " << endl;
    str2.erase(0, 2);
    cout << str2.data() << " ";
    str2.replace(0, 2, "la");
    cout << str2.data() << endl;
    str2.clear();
    cout << str2.data() << endl;
    return 0;
}
