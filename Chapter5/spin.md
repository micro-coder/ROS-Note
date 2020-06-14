# 回调函数与spin()函数

回调函数在编程中是一种重要的方法，在维基百科上的解释是：  
In computer programming, a callback is any executable code that is passed as an argument to other code, which is expected to call back (execute) the argument at a given time.  
回调函数作为参数被传入到了另一个函数中（在本例中传递的是函数指针），在未来某个时刻（当有新的message到达），就会立即执行。Subscriber接收到消息，实际上是先把消息放到一个接受队列中去，如图所示。队列的长度在Subscriber构建的时候设置。当有spin函数执行，就会去处理消息队列中队首的消息。
![cb_queue.png](picture/cb_queue.png)

spin具体处理的方法又可分为阻塞/非阻塞,单线程/多线程，在ROS函数接口层面我们有4种spin的方式：

spin方法|	是否阻塞|	线程
:---:|:---:
ros::spin()	|阻塞	|单线程
ros::spinOnce()	|非阻塞	|单线程
ros::MultiThreadedSpin()	|阻塞|	多线程
ros::AsyncMultiThreadedSpin()|	非阻塞|	多线程

阻塞与非阻塞的区别我们已经讲了，下面来看看单线程与多线程的区别：
![single-multi-spin.png](picture/single-multi-spin.png)

我们常用的spin()、spinOnce()是单个线程逐个处理回调队列里的数据。有些场合需要用到多线程分别处理，则可以用到MultiThreadedSpin()、AsyncMultiThreadedSpin()。