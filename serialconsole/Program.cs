using System;
using System.IO.Ports;

namespace weather
{
	class MainClass
	{
		public static void Main()
		{
			string ttyname = "";
			string ReceivedData = "";

			SerialPort Serial_tty = new SerialPort();
			string acmport = string.Empty;

			// search the ACM port to which the station is connected
			while (acmport == string.Empty)
			{
				string[] ports = SerialPort.GetPortNames ();
				foreach (string port in ports)
				{
					if (port.ToLower().StartsWith("/dev/ttyacm"))
					{
						acmport = port;
						break;
					}
				}
			}

			Console.WriteLine("Waiting for station to start up...");

			ttyname = acmport;
			Serial_tty.PortName = ttyname;
			Serial_tty.BaudRate = 115200;             // Baudrate = 115200 bps default
			Serial_tty.Parity   = Parity.None;        // Parity bits = none  
			Serial_tty.DataBits = 8;                  // No of Data bits = 8
			Serial_tty.StopBits = StopBits.One;       // No of Stop bits = 1

			const string welcome = "OpenObservatory v3.0 now online!";
			bool started = false;

			try
			{
				
				Serial_tty.Open();
				Serial_tty.ReadExisting(); // clear existing data in buffer

				// continuously log data
				while (true)
				{
					ReceivedData = Serial_tty.ReadLine();
					if (started) Console.WriteLine(ReceivedData);

					// wait for the station to send a welcome message
					if (ReceivedData.Trim() == welcome && !started)
					{
						Console.WriteLine("Detected WeaterStation v3.0 on port {0}", acmport);
						started = true;
					}
				}
			}
			catch
			{
				Console.WriteLine("Error in Opening {0}\n",Serial_tty.PortName);
			}
		}
	}
}
