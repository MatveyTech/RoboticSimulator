#!/usr/bin/env python
import pika

cred = pika.credentials.PlainCredentials(username="backbone", password="backbone", erase_on_connect=False)
connection = pika.BlockingConnection(pika.ConnectionParameters(host="192.168.58.64",port="5672",credentials=cred))
channel = connection.channel()

channel.queue_declare(queue='hello')
channel.basic_publish(exchange='', routing_key='hello', body='Hello2 World!')
print(" [x] Sent 'Hello2 World!'")
connection.close()