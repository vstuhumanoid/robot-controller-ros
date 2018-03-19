#include <iostream>

using namespace std;

class Wtf
{
public:
    Wtf() { cout << "Constructor\n"; }
    ~Wtf() { cout << "Desctructor\n"; }
    Wtf(const Wtf& w) { cout << "Copy constructor\n"; }
    Wtf& operator=(const Wtf& w) { cout << "Copy =\n"; }
    Wtf(Wtf&& w) { cout << "Move constructor\n"; }
    Wtf& operator=(Wtf&& w){ cout << "Move =\n"; }

    void bar() { cout << "bar\n"; }
};

Wtf foo1()
{
    Wtf w;
    return w;
}

int main(int argc, char** argv)
{
    /* Constructor
     * Descturctor */
    //cout << "1\n";
    //Wtf w1;

    /* Constructor
     * Descturctor */
    //cout << "2\n";
    //Wtf w2 = Wtf();


    /* Constructor
     * Constructor
     * Move =
     * Destructor
     * Destructor
     */
    //cout << "3\n";
    //Wtf w3;
    //w3 = Wtf();

    /* Constructor
     * Destructor */
    //foo1();

    Wtf w = foo1();
    w.bar();

    return 0;

}