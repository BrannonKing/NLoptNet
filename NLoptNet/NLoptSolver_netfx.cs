using System;
using System.Runtime.InteropServices;

namespace NLoptNet
{
    public partial class NLoptSolver
    {
#if NETFRAMEWORK
#if _WINDOWS
		[DllImport("kernel32", SetLastError = true)]
		private static extern IntPtr LoadLibrary(string lpFileName);

		static NLoptSolver()
		{
			var path = IntPtr.Size > 4 ? "win-x64" : "win-x86";
			LoadLibrary($"runtimes/{path}/native/nlopt.dll");
		}
#endif
#if _LINUX64
		static NLoptSolver()
		{
			Environment.SetEnvironmentVariable("LD_LIBRARY_PATH", "runtimes/linux-x64/native");
		}
#endif
#endif
    }
}