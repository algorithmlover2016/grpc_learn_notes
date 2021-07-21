// #define ASYNC_CALL_NEED_WAIT_IN_CALLING
#define ASYNC_DEPENDENCY

#if !ASYNC_CALL_NEED_WAIT_IN_CALLING && (!ASYNC_DEPENDENCY)
#define ASYNC_AWAIT_CALL_WITH_ASYNC
#endif

#if ASYNC_DEPENDENCY
#define BUG_CALL_USING
/*
the task.Wait is not make sense.
 */
#endif
#undef BUG_CALL_USING

using System;
using System.IO;
using System.Threading.Tasks;

// reference to https://www.c-sharpcorner.com/article/async-and-await-in-c-sharp/
// reference to https://blog.csdn.net/lindexi_gd/article/details/100175014


namespace AsyncAwaitTest
{
    class Program
    {
#if ASYNC_CALL_NEED_WAIT_IN_CALLING
        static void Main(string[] args)
        {
            Console.WriteLine("Hello AsyncAwaitTest!");
            var ele = Method1();
            Method2();
            Console.ReadKey(); // just read a character to wait the asynchronus.
            ele.Wait();
        }
        public static async Task Method1()
        {
            await Task.Run(
                () =>
                {
                    for (int i = 0; i < 100; i++)
                    {
                        Console.WriteLine("Method 1");
                        Task.Delay(100).Wait();
                    }

                }
                );
        }

        public static void Method2()
        {
            for (int i = 0; i < 20; i++)
            {
                Console.WriteLine("Method 2");
                Task.Delay(10).Wait();
            }

        }

#elif ASYNC_AWAIT_CALL_WITH_ASYNC
        static void Main(string[] args)
        {

            var task = callMethod();
            task.Wait();
        }
        static async Task callMethod()
        {
            Task<int> task = Method1();
            Method2();
            int count = await task;
            int cnt = await Method1();
            Method3(count);
            Method3(cnt);
        }
        public static async Task<int> Method1()
        {
            int count = 0;
            await Task.Run(() =>
            {
                for (int i = 0; i < 100; i++)
                {
                    Console.WriteLine(" Method 1");
                    count += 1;
                }
            });
            return count;
        }

        public static void Method2()
        {
            for (int i = 0; i < 25; i++)
            {
                Console.WriteLine(" Method 2");
            }
        }

        public static void Method3(int count)
        {
            Console.WriteLine("Total count is " + count);
        }


#else
        static void Main()
        {
#if BUG_CALL_USING
            Task task = new Task(CallMethod);
            task.Start();
#else
            var task = CallMethod();
#endif
            task.Wait(); // if CallMethod doesn't return Task, the Wait will be not in effect.
            Console.ReadLine();
        }

#if BUG_CALL_USING
         static async void CallMethod()
#else
        static async Task CallMethod()
#endif
        {
            string filePath = @"D:\littleStar\tmp\video_play_data\ingest_data_into_kusto_11_6400__622_626_03";
            Task<int> task = ReadFile(filePath);

            Console.WriteLine(" Other Work 1");
            Console.WriteLine(" Other Work 2");
            Console.WriteLine(" Other Work 3");

            int length = await task;
            Console.WriteLine(" Total length: " + length);

            Console.WriteLine(" After work 1");
            Console.WriteLine(" After work 2");
        }

        static async Task<int> ReadFile(string file)
        {
            int length = 0;

            Console.WriteLine(" File reading is starting");
            using (StreamReader reader = new StreamReader(file))
            {
                // Reads all characters from the current position to the end of the stream asynchronously
                // and returns them as one string.
                Console.WriteLine("DEBUG File reading is async");
                string s = await reader.ReadToEndAsync();
                Console.WriteLine("DEBUG File reading is get len");

                length = s.Length;
                Console.WriteLine("DEBUG File reading is done");
            }
            Console.WriteLine(" File reading is completed");
            return length;
        }
#endif
        }

}
