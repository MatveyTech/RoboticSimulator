using RabbitMQ.Client;
using RabbitMQ.Client.Events;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CS_GUI
{
    public class RabbitService 
    {
        static private string VmExchange = "";

        static private IModel Channel = null;
        static private IConnection m_connection = null;

        static RabbitService()
        {
            RecreateConnection();
        }


        //static public void Publish(string topic, object param)
        //{
        //    Publish(topic, param);
        //}
        public string Subscribe(string topicKey, Action<string, string> callback)
        {
            if (string.IsNullOrEmpty(topicKey))
            {
                return string.Empty;
            }

            try
            {
                return EstablishQueue(topicKey, callback);
            }
            catch (RabbitMQ.Client.Exceptions.AlreadyClosedException)
            {
                return string.Empty;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.Assert(false, "RabbitMQ Connection Error", ex.ToString());
                throw;
            }
        }
        static public void RecreateConnection()
        {
            var factory = new ConnectionFactory()
            {
                HostName = "192.168.58.64",
                Port = 5672,
                UserName = "backbone",
                Password = "backbone"
            };
            m_connection = factory.CreateConnection();
            Channel = m_connection.CreateModel();
            //Channel.ExchangeDeclare(exchange: VmExchange, type: ExchangeType.Topic);
            Channel.QueueDeclare("hello", autoDelete:false, exclusive:false);
        }

        static public IModel GetChannel()
        {
            return Channel;
        }

        static public void Publish(string topic, string message)
        {
            if (string.IsNullOrEmpty(topic)) { return; }

            //Debug.Assert(Channel.IsClosed == false, "Channle was closed");

            byte[] messageBytes = Encoding.UTF8.GetBytes(message);
            GetChannel().BasicPublish(exchange: VmExchange,
                                  routingKey: topic,
                                  basicProperties: null,
                                  body: messageBytes);
        }
        private string EstablishQueue(string topicKey, Action<string, string> callback)
        {
            var queue = Channel.QueueDeclare();
            var queueName = queue.QueueName;
            GetChannel().QueueBind(queue: queueName,
                               exchange: VmExchange,
                               routingKey: topicKey);

            var consumer = new EventingBasicConsumer(Channel);
            consumer.Received += (model, ea) =>
            {
                var topic = ea.RoutingKey;
                var body = Encoding.UTF8.GetString(ea.Body);
            };
            return GetChannel().BasicConsume(queue: queueName, autoAck: true, consumer: consumer);
        }
    }

}
