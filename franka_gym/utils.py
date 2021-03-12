import genpy
import rospy

from typing import Optional, Type, Callable

Message = genpy.Message
Dataclass = Type[genpy.Message]


def has_subscriber(pub: rospy.Publisher) -> bool:
    """Whether the publisher has a least one subscriber

    Args:
        pub (rospy.Publisher): The publisher you want to know if it has
            a subscriber

    Returns:
        bool: `True` if publisher has at least one subscriber.
    """
    return pub.get_num_connections() > 0


def publisher(
        name: str, data_class: Dataclass,
        timeout: Optional[float] = None) -> rospy.Publisher:
    """Registering as a publisher of a ROS topic.

    Args:
        name (str): Topic name.
        data_class (Type[Message]): Data class of the topic.
        timeout (float, optional): Raises an error if no subscriber is detected
            on the topic after `timeout` seconds. If `None`, waits without
            timeout. If `-1`, does not wait for a message to be published.
            Defaults to `None`.

    Raises:
        ROSInterruptException: If waiting is interrupted.
        TimeoutError: If no subscriber is detected on the topic after `timeout`
            seconds. Set `timeout` to `None` to wait endlessly. Set `timeout` to
            `-1` not to wait for a subscriber.

    Returns:
        Publisher: The publisher. To publish a message, call its method
            `publish(your_msg)`.
    """
    start = rospy.Time.now()
    pub = rospy.Publisher(name, data_class, queue_size=10)
    if timeout == -1:  # do not wait for subscriber if timeout = -1
        return pub
    rospy.logdebug('Looking for subscriber for topic %s' % name)
    while not has_subscriber(pub):  # while no subscriber
        if timeout is not None and rospy.Time.now() - start > rospy.Duration(secs=timeout):
            err_msg = 'Timeout execeded, no subscriber '\
                'found for topic %s' % name
            rospy.logerr(err_msg)
            raise TimeoutError(err_msg)
        else:
            try:
                rospy.sleep(1/5.)
            except rospy.ROSTimeMovedBackwardsException:
                # To avoid error when world is rested, time when backwards.
                rospy.logdebug('Time moved backward, ignoring')
            except rospy.ROSInterruptException as e:
                rospy.logdebug(
                    'Waiting for subscriber for topic %s interrupted' % name)
                raise e

    rospy.logdebug('Subscriber found for topic %s, publisher connected' % name)
    return pub


def subscriber(
        name: str, data_class: Dataclass, callback: Callable[[Message], None],
        timeout: Optional[float] = None, tcp_nodelay=False) -> rospy.Subscriber:
    """Registering as a subscriber to a topic.

    Args:
        name (str): Topic name.
        data_class (Type[Message]): Data class of the topic.
        callback (Callable[[Message], None]): The function called each
            time a message is published on the topic.
        timeout (float, optional): Raise error if no message is published on
            the topic after a time. If `None`, waits without timeout. If `-1`,
            does not wait for a message to be published. Defaults to `None`.
        tcp_nodelay (bool, optional): if `True`, request TCP_NODELAY from
            publisher. More details in rospy.Subscriber docstring.

    Raises:
        ROSInterruptException: If waiting is interrupted.
        TimeoutError: If no message is published on the topic after
          `timeout` seconds. Set `timeout` to `None` not to raise this error.

    Returns:
        Subscriber: The subscriber. To stop subscription, call its method
            `unregister()`.
    """
    if timeout != -1: # wait for a message to be received if timeout != 1
        try:
            rospy.logdebug('Waiting for message on topic %s' % name)
            rospy.wait_for_message(name, data_class, timeout)
        except rospy.ROSInterruptException as e:
            rospy.logdebug(
                'Waiting for topic %s interrupted' % name)
            raise e
        except rospy.ROSException as e:
            print(e)
            err_msg = 'Timeout exceded, no message received on topic %s' % name
            rospy.logerr(err_msg)
            raise TimeoutError(err_msg)

    rospy.logdebug('Subscriber to %s ready' % (name))
    return rospy.Subscriber(name, data_class, callback,
                            tcp_nodelay=tcp_nodelay)


if __name__ == '__main__':
    import time
    from geometry_msgs.msg import Twist
    from turtlesim.msg import Pose

    rospy.init_node('my_node', anonymous=True, disable_signals=True) # disable signal to allow KeyboardInterrupt

    show = lambda msg: print(msg)

    pub = publisher('/turtle1/cmd_vel', Twist, timeout=3)
    sub = subscriber('/turtle1/pose', Pose, show, timeout=-1)

    vel_msg = Twist()
    vel_msg.angular.z=1

    rate = rospy.Rate(1)
    pub.publish(vel_msg) # first message, won't be processed
    rate.sleep() # node need a few time to process the first message
    pub.publish(vel_msg)



    

