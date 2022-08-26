using System;
using System.Runtime.InteropServices;

namespace NLoptNet
{
	public partial class NLoptSolver
	{
		private static bool Is64Bit => IntPtr.Size == 8;
		private const string Subscript32 = @"_x32";
		private const string Subscript64 = @"_x64";
		private static string Subscript => Is64Bit ? Subscript64 : Subscript32;
		private const string NLopt32 = "nlopt_x32.dll";
		private const string NLopt64 = "nlopt_x64.dll";

		#region Imported NLOpt functions - .NET Framework
		
		#if NETFRAMEWORK
		
		[DllImport(NLopt64, EntryPoint = nameof(nlopt_version), CallingConvention = CallingConvention.Cdecl)]
		private static extern void nlopt_version64(out int major, out int minor, out int bugfix);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_version), CallingConvention = CallingConvention.Cdecl)]
		private static extern void nlopt_version32(out int major, out int minor, out int bugfix);
		
		private static void nlopt_version(out int major, out int minor, out int bugfix)
		{
			if (Is64Bit)
			{
				nlopt_version64(out major, out minor, out bugfix);
				return;
			}

			nlopt_version32(out major, out minor, out bugfix);
		}

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_create), CallingConvention = CallingConvention.Cdecl)]
		private static extern IntPtr nlopt_create64(NLoptAlgorithm algorithm, uint n);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_create), CallingConvention = CallingConvention.Cdecl)]
		private static extern IntPtr nlopt_create32(NLoptAlgorithm algorithm, uint n);

		private static IntPtr nlopt_create(NLoptAlgorithm algorithm, uint n)
			=> Is64Bit ? nlopt_create64(algorithm, n) : nlopt_create32(algorithm, n);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_destroy), CallingConvention = CallingConvention.Cdecl)]
		private static extern void nlopt_destroy64(IntPtr opt);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_destroy), CallingConvention = CallingConvention.Cdecl)]
		private static extern void nlopt_destroy32(IntPtr opt);

		private static void nlopt_destroy(IntPtr opt)
		{
			if (Is64Bit)
			{
				nlopt_destroy64(opt);
				return;
			}

			nlopt_destroy32(opt);
		}

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_optimize), CallingConvention = CallingConvention.Cdecl, SetLastError = true)]
		private static extern NloptResult nlopt_optimize64(IntPtr opt, double[] x, ref double result);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_optimize), CallingConvention = CallingConvention.Cdecl, SetLastError = true)]
		private static extern NloptResult nlopt_optimize32(IntPtr opt, double[] x, ref double result);

		private static NloptResult nlopt_optimize(IntPtr opt, double[] x, ref double result)
			=> Is64Bit ? nlopt_optimize64(opt, x, ref result) : nlopt_optimize32(opt, x, ref result);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_min_objective), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_min_objective64(IntPtr opt, nlopt_func f, IntPtr data);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_set_min_objective), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_min_objective32(IntPtr opt, nlopt_func f, IntPtr data);

		private static NloptResult nlopt_set_min_objective(IntPtr opt, nlopt_func f, IntPtr data)
			=> Is64Bit ? nlopt_set_min_objective64(opt, f, data) : nlopt_set_min_objective32(opt, f, data);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_max_objective), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_max_objective64(IntPtr opt, nlopt_func f, IntPtr data);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_set_max_objective), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_max_objective32(IntPtr opt, nlopt_func f, IntPtr data);

		private static NloptResult nlopt_set_max_objective(IntPtr opt, nlopt_func f, IntPtr data)
			=> Is64Bit ? nlopt_set_max_objective64(opt, f, data) : nlopt_set_max_objective32(opt, f, data);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_get_algorithm), CallingConvention = CallingConvention.Cdecl)]
		private static extern NLoptAlgorithm nlopt_get_algorithm64(IntPtr opt);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_get_algorithm), CallingConvention = CallingConvention.Cdecl)]
		private static extern NLoptAlgorithm nlopt_get_algorithm32(IntPtr opt);

		private static NLoptAlgorithm nlopt_get_algorithm(IntPtr opt)
			=> Is64Bit ? nlopt_get_algorithm64(opt) : nlopt_get_algorithm32(opt);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_get_dimension), CallingConvention = CallingConvention.Cdecl)]
		private static extern uint nlopt_get_dimension64(IntPtr opt);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_get_dimension), CallingConvention = CallingConvention.Cdecl)]
		private static extern uint nlopt_get_dimension32(IntPtr opt);

		private static uint nlopt_get_dimension(IntPtr opt)
			=> Is64Bit ? nlopt_get_dimension64(opt) : nlopt_get_dimension32(opt);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_lower_bounds), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_lower_bounds64(IntPtr opt, double[] lowerBounds);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_set_lower_bounds), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_lower_bounds32(IntPtr opt, double[] lowerBounds);

		private static NloptResult nlopt_set_lower_bounds(IntPtr opt, double[] lowerBounds)
			=> Is64Bit ? nlopt_set_lower_bounds64(opt, lowerBounds) : nlopt_set_lower_bounds32(opt, lowerBounds);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_upper_bounds), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_upper_bounds64(IntPtr opt, double[] upperBounds);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_set_upper_bounds), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_upper_bounds32(IntPtr opt, double[] upperBounds);

		private static NloptResult nlopt_set_upper_bounds(IntPtr opt, double[] upperBounds)
			=> Is64Bit ? nlopt_set_upper_bounds64(opt, upperBounds) : nlopt_set_upper_bounds32(opt, upperBounds);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_add_inequality_constraint), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_add_inequality_constraint64(IntPtr opt, nlopt_func fc, IntPtr data, double tolerance);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_add_inequality_constraint), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_add_inequality_constraint32(IntPtr opt, nlopt_func fc, IntPtr data, double tolerance);

		private static NloptResult nlopt_add_inequality_constraint(IntPtr opt, nlopt_func fc, IntPtr data, double tolerance)
			=> Is64Bit ? nlopt_add_inequality_constraint64(opt, fc, data, tolerance) : nlopt_add_inequality_constraint32(opt, fc, data, tolerance);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_add_equality_constraint), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_add_equality_constraint64(IntPtr opt, nlopt_func fc, IntPtr data, double tolerance);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_add_equality_constraint), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_add_equality_constraint32(IntPtr opt, nlopt_func fc, IntPtr data, double tolerance);

		private static NloptResult nlopt_add_equality_constraint(IntPtr opt, nlopt_func fc, IntPtr data, double tolerance)
			=> Is64Bit ? nlopt_add_equality_constraint64(opt, fc, data, tolerance) : nlopt_add_equality_constraint32(opt, fc, data, tolerance);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_add_equality_mconstraint), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_add_equality_mconstraint64(IntPtr opt, uint m, nlopt_mfunc fc, IntPtr data, double[] tolerances);
		
		[DllImport(NLopt32, EntryPoint = nameof(nlopt_add_equality_mconstraint), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_add_equality_mconstraint32(IntPtr opt, uint m, nlopt_mfunc fc, IntPtr data, double[] tolerances);
		
		private static NloptResult nlopt_add_equality_mconstraint(IntPtr opt, uint m, nlopt_mfunc fc, IntPtr data, double[] tolerances)
			=> Is64Bit ? nlopt_add_equality_mconstraint64(opt, m, fc, data, tolerances) : nlopt_add_equality_mconstraint32(opt, m, fc, data, tolerances);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_add_inequality_mconstraint), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_add_inequality_mconstraint64(IntPtr opt, uint m, nlopt_mfunc fc, IntPtr data, double[] tolerances);
		
		[DllImport(NLopt32, EntryPoint = nameof(nlopt_add_inequality_mconstraint), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_add_inequality_mconstraint32(IntPtr opt, uint m, nlopt_mfunc fc, IntPtr data, double[] tolerances);
		
		private static NloptResult nlopt_add_inequality_mconstraint(IntPtr opt, uint m, nlopt_mfunc fc, IntPtr data, double[] tolerances)
			=> Is64Bit ? nlopt_add_inequality_mconstraint64(opt, m, fc, data, tolerances) : nlopt_add_inequality_mconstraint32(opt, m, fc, data, tolerances);
		
		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_xtol_rel), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_xtol_rel64(IntPtr opt, double tolerance);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_set_xtol_rel), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_xtol_rel32(IntPtr opt, double tolerance);

		private static NloptResult nlopt_set_xtol_rel(IntPtr opt, double tolerance)
			=> Is64Bit ? nlopt_set_xtol_rel64(opt, tolerance) : nlopt_set_xtol_rel32(opt, tolerance);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_local_optimizer), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_local_optimizer64(IntPtr opt, IntPtr local);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_set_local_optimizer), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_local_optimizer32(IntPtr opt, IntPtr local);

		private static NloptResult nlopt_set_local_optimizer(IntPtr opt, IntPtr local)
			=> Is64Bit ? nlopt_set_local_optimizer64(opt, local) : nlopt_set_local_optimizer32(opt, local);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_maxeval), CallingConvention = CallingConvention.Cdecl)]
		private static extern void nlopt_set_maxeval64(IntPtr opt, int maxeval);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_set_maxeval), CallingConvention = CallingConvention.Cdecl)]
		private static extern void nlopt_set_maxeval32(IntPtr opt, int maxeval);

		private static void nlopt_set_maxeval(IntPtr opt, int maxeval)
		{
			if (Is64Bit)
			{
				nlopt_set_maxeval64(opt, maxeval);
				return;
			}

			nlopt_set_maxeval32(opt, maxeval);
		}
		
		[DllImport(NLopt64, EntryPoint = nameof(nlopt_force_stop), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_force_stop64(IntPtr opt);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_force_stop), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_force_stop32(IntPtr opt);

		private static NloptResult nlopt_force_stop(IntPtr opt)
		{
			return Is64Bit ? nlopt_force_stop64(opt) : nlopt_force_stop32(opt);
		}
	        
		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_initial_step), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_initial_step64(IntPtr opt, double[] dx);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_set_initial_step), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_initial_step32(IntPtr opt, double[] dx);

		private static NloptResult nlopt_set_initial_step(IntPtr opt, double[] dx)
			=> Is64Bit ? nlopt_set_initial_step64(opt, dx) : nlopt_set_initial_step32(opt, dx);

		[DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_initial_step1(IntPtr opt, double dx);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_ftol_rel), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_ftol_rel64(IntPtr opt, double tol);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_set_ftol_rel), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_ftol_rel32(IntPtr opt, double tol);

		private static NloptResult nlopt_set_ftol_rel(IntPtr opt, double tol)
			=> Is64Bit ? nlopt_set_ftol_rel64(opt, tol) : nlopt_set_ftol_rel32(opt, tol);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_ftol_abs), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_ftol_abs64(IntPtr opt, double tol);

		[DllImport(NLopt32, EntryPoint = nameof(nlopt_set_ftol_abs), CallingConvention = CallingConvention.Cdecl)]
		private static extern NloptResult nlopt_set_ftol_abs32(IntPtr opt, double tol);

		private static NloptResult nlopt_set_ftol_abs(IntPtr opt, double tol)
			=> Is64Bit ? nlopt_set_ftol_abs64(opt, tol) : nlopt_set_ftol_abs32(opt, tol);

		[DllImport(NLopt64, EntryPoint = nameof(nlopt_set_xtol_abs1), CallingConvention = CallingConvention.Cdecl)]
	    private static extern NloptResult nlopt_set_xtol_abs164(IntPtr opt, double tol);

	    [DllImport(NLopt32, EntryPoint = nameof(nlopt_set_xtol_abs1), CallingConvention = CallingConvention.Cdecl)]
	    private static extern NloptResult nlopt_set_xtol_abs132(IntPtr opt, double tol);

	    private static NloptResult nlopt_set_xtol_abs1(IntPtr opt, double tol)
		    => Is64Bit ? nlopt_set_xtol_abs164(opt, tol) : nlopt_set_xtol_abs132(opt, tol);

	    [DllImport(NLopt64, EntryPoint = nameof(nlopt_set_xtol_abs), CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_xtol_abs64(IntPtr opt, double tol);

        [DllImport(NLopt32, EntryPoint = nameof(nlopt_set_xtol_abs), CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_xtol_abs32(IntPtr opt, double tol);

        private static NloptResult nlopt_set_xtol_abs(IntPtr opt, double tol)
	        => Is64Bit ? nlopt_set_xtol_abs64(opt, tol) : nlopt_set_xtol_abs32(opt, tol);

        [DllImport(NLopt64, EntryPoint = nameof(nlopt_get_ftol_rel), CallingConvention = CallingConvention.Cdecl)]
		private static extern double nlopt_get_ftol_rel64(IntPtr opt);

	    [DllImport(NLopt32, EntryPoint = nameof(nlopt_get_ftol_rel), CallingConvention = CallingConvention.Cdecl)]
	    private static extern double nlopt_get_ftol_rel32(IntPtr opt);

	    private static double nlopt_get_ftol_rel(IntPtr opt)
		    => Is64Bit ? nlopt_get_ftol_rel64(opt) : nlopt_get_ftol_rel32(opt);

	    [DllImport(NLopt64, EntryPoint = nameof(nlopt_get_ftol_abs), CallingConvention = CallingConvention.Cdecl)]
	    private static extern double nlopt_get_ftol_abs64(IntPtr opt);

	    [DllImport(NLopt32, EntryPoint = nameof(nlopt_get_ftol_abs), CallingConvention = CallingConvention.Cdecl)]
	    private static extern double nlopt_get_ftol_abs32(IntPtr opt);

	    private static double nlopt_get_ftol_abs(IntPtr opt)
		    => Is64Bit ? nlopt_get_ftol_abs64(opt) : nlopt_get_ftol_abs32(opt);

	    [DllImport(NLopt64, EntryPoint = nameof(nlopt_get_xtol_rel), CallingConvention = CallingConvention.Cdecl)]
	    private static extern double nlopt_get_xtol_rel64(IntPtr opt);

	    [DllImport(NLopt32, EntryPoint = nameof(nlopt_get_xtol_rel), CallingConvention = CallingConvention.Cdecl)]
	    private static extern double nlopt_get_xtol_rel32(IntPtr opt);

	    private static double nlopt_get_xtol_rel(IntPtr opt)
		    => Is64Bit ? nlopt_get_xtol_rel64(opt) : nlopt_get_xtol_rel32(opt);

	    [DllImport(NLopt64, EntryPoint = nameof(nlopt_get_xtol_abs), CallingConvention = CallingConvention.Cdecl)]
        private static extern double nlopt_get_xtol_abs64(IntPtr opt);

        [DllImport(NLopt32, EntryPoint = nameof(nlopt_get_xtol_abs), CallingConvention = CallingConvention.Cdecl)]
        private static extern double nlopt_get_xtol_abs32(IntPtr opt);

        private static double nlopt_get_xtol_abs(IntPtr opt)
	        => Is64Bit ? nlopt_get_xtol_abs64(opt) : nlopt_get_xtol_abs32(opt);

        #endif
        
        #endregion
	}
}