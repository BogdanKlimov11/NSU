class FirstClass {
    
    synchronized void foo(SecondClass b) {
        String name = Thread.currentThread().getName();
        System.out.println(name + " in FirstClass.foo()");
        try {
            Thread.sleep(1000);
        } catch (Exception e) {} 
        System.out.println(name + " try to execute SecondClass.last()");
        b.last();
    }
    synchronized void last() { System.out.println("FirstClass.last()"); }
}
 
class SecondClass {

    synchronized void bar(FirstClass a) {
        String name = Thread.currentThread().getName();
        System.out.println(name + " in SecondClass.bar()");
 	try {
            Thread.sleep(1000);
	} catch (Exception e) {}
	System.out.println(name + " try to execute FirstClass.last()");
        a.last();
    }
    synchronized void last() { System.out.println("SecondClass.last()"); }
}
 
class Deadlock implements Runnable {
    FirstClass a = new FirstClass();
    SecondClass b = new SecondClass();

    Deadlock() {
	Thread.currentThread().setName("Main Stream");
	Thread t = new Thread(this, "Second Stream");
	t.start();
	a.foo(b);
    }
 
    public void run() {
	b.bar(a); 
    }
 
    public static void main(String args[]) {
        new Deadlock();
    }
}
