#!/usr/bin/env python
import pika

exchange = "stealth"

cred = pika.credentials.PlainCredentials(username="backbone", password="backbone", erase_on_connect=False)
connection = pika.BlockingConnection(pika.ConnectionParameters(host="192.168.58.64",port="5672",credentials=cred))
channel = connection.channel()

channel.queue_declare(queue='q1',auto_delete=True)
q = channel.queue_bind("q1", exchange, "topic1")
#print (q)

def callback(ch, method, properties, body):
    print (ch)
    print (method)
    print (properties)
    print (body)
    print(" [x] Received %r" % body)


#channel.basic_consume(queue='hello', on_message_callback=callback, auto_ack=True)
channel.basic_consume(queue='q1', on_message_callback=callback, auto_ack=True)


print(' [*] Waiting for messages. To exit press CTRL+C')
channel.start_consuming()
#queueName = queue.QueueName;
#            GetChannel().QueueBind(queue: queueName,
#                               exchange: VmExchange,
#                               routingKey: topicKey);
#
#            var consumer = new EventingBasicConsumer(Channel);
#            consumer.Received += (model, ea) =>
#            {
#                var topic = ea.RoutingKey;
#                var body = Encoding.UTF8.GetString(ea.Body);
#            };
#            return GetChannel().BasicConsume(queue: queueName, autoAck: true, consumer: consumer);
#
#
#
#
##channel.queue_declare(queue='hello2',auto_delete=True)
#channel.exchange_declare(exchange="mt",exchange_type="topic")
#
#
#def callback(ch, method, properties, body):
#    print (ch)
#    print (method)
#    print (properties)
#    print (body)
#    print(" [x] Received %r" % body)
#
#
##channel.basic_consume(queue='hello', on_message_callback=callback, auto_ack=True)
#channel.basic_consume(queue='hello2', on_message_callback=callback, auto_ack=True)
#
#
#print(' [*] Waiting for messages. To exit press CTRL+C')
#channel.start_consuming()





##!/usr/bin/env python
#import pika
#
#cred = pika.credentials.PlainCredentials(username="backbone", password="backbone", erase_on_connect=False)
#connection = pika.BlockingConnection(pika.ConnectionParameters(host="192.168.58.64",port="5672",credentials=cred))
#channel = connection.channel()
#
##channel.queue_declare(queue='hello2',auto_delete=True)
#channel.exchange_declare(exchange="mt",exchange_type="topic")
#
#
#def callback(ch, method, properties, body):
#    print (ch)
#    print (method)
#    print (properties)
#    print (body)
#    print(" [x] Received %r" % body)
#
#
##channel.basic_consume(queue='hello', on_message_callback=callback, auto_ack=True)
#channel.basic_consume(queue='hello2', on_message_callback=callback, auto_ack=True)
#
#
#print(' [*] Waiting for messages. To exit press CTRL+C')
#channel.start_consuming()