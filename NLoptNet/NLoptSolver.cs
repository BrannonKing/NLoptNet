#if (__MonoCS__ || (UNITY_5_3_OR_NEWER && ENABLE_MONO))
// Detect if mono is used
#define MONO
#endif

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace NLoptNet
{
	/// <summary>
	/// This class wraps the NLopt C library. Be sure to dispose it when done.
	/// </summary>
	public partial class NLoptSolver : IDisposable
	{
#if MONO
				[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
				private delegate double nlopt_func(
					uint n,
					IntPtr p_x,
					IntPtr p_gradient,
					IntPtr data
				);
#else
		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		private delegate double nlopt_func(
			uint n,
			[MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 0), In] double[] x,
			[MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 0), In, Out] double[] gradient,
			IntPtr data
		);
#endif

#if MONO
		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		private delegate void nlopt_mfunc(
			uint m,
			IntPtr p_result,
			uint n,
			IntPtr p_x,
			IntPtr p_gradient,
			IntPtr data
		);
#else
		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		private delegate void nlopt_mfunc(
			uint m,
			[MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 0), In, Out] double[] result,
			uint n,
			[MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2), In] double[] x,
			[MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2), In, Out] double[] gradient,
			IntPtr data
		);
#endif

		private IntPtr _opt;
		private readonly List<(Delegate, nlopt_func)> _funcCache = new List<(Delegate, nlopt_func)>();
		private readonly Dictionary<Delegate, nlopt_mfunc> _mfuncCache = new Dictionary<Delegate, nlopt_mfunc>();

		public NLoptSolver(NLoptAlgorithm algorithm, uint numVariables, double relativeStoppingTolerance = 0.0001, int maximumIterations = 0, NLoptAlgorithm? childAlgorithm = null)
		{
			if (numVariables < 1)
				throw new ArgumentOutOfRangeException("numVariables");

			_opt = nlopt_create(algorithm, numVariables);
			if (_opt == IntPtr.Zero)
				throw new ArgumentException("Unable to initialize the algorithm.", "algorithm");

			if (relativeStoppingTolerance > 0.0)
			{
				var res = nlopt_set_xtol_rel(_opt, relativeStoppingTolerance);
				if (res != NloptResult.SUCCESS)
					throw new ArgumentException("Unable to set primary tolerance. Result: " + res, "relativeStoppingTolerance");
			}

			if (maximumIterations > 0)
				nlopt_set_maxeval(_opt, maximumIterations);

			if (childAlgorithm != null)
			{
				var inner = nlopt_create(childAlgorithm.Value, numVariables);
				if (inner == IntPtr.Zero)
					throw new ArgumentException("Unable to initialize the secondary algorithm.", "childAlgorithm");

				if (relativeStoppingTolerance > 0.0)
				{
					var res = nlopt_set_xtol_rel(inner, relativeStoppingTolerance);
					if (res != NloptResult.SUCCESS)
						throw new ArgumentException("Unable to set secondary tolerance. Result: " + res, "relativeStoppingTolerance");
				}

				if (maximumIterations > 0)
					nlopt_set_maxeval(inner, maximumIterations);

				var ret = nlopt_set_local_optimizer(_opt, inner);
				nlopt_destroy(inner);

				if (ret != NloptResult.SUCCESS)
					throw new ArgumentException("Unable to associate the child optimizer. Result: " + ret, "childAlgorithm");
			}
		}

		/// <summary>
		/// Primary/outer algorithm.
		/// </summary>
		public NLoptAlgorithm Algorithm { get { return nlopt_get_algorithm(_opt); } }

		/// <summary>
		/// Number of variables.
		/// </summary>
		public uint Dimension { get { return nlopt_get_dimension(_opt); } }

		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}

		~NLoptSolver()
		{
			Dispose(false);
		}

		protected virtual void Dispose(bool isManaged)
		{
			if (isManaged)
				_funcCache.Clear();
				_mfuncCache.Clear();

			if (_opt != IntPtr.Zero)
			{
				nlopt_destroy(_opt);
				_opt = IntPtr.Zero;
			}
		}

		/// <summary>
		/// Check if an inequality constaint can be applied on current algorithm.
		/// In NLopt, api/options.c has a function inequality_ok() which do the same verification.
		/// </summary>
		protected void CheckInequalityConstraintAvailability()
		{
			NLoptAlgorithm algorithm = Algorithm;
			switch (algorithm)
			{
				case NLoptAlgorithm.LD_MMA:
				case NLoptAlgorithm.LD_CCSAQ:
				case NLoptAlgorithm.LD_SLSQP:
				case NLoptAlgorithm.LN_COBYLA:
				case NLoptAlgorithm.GN_ISRES:
				case NLoptAlgorithm.GN_ORIG_DIRECT:
				case NLoptAlgorithm.GN_ORIG_DIRECT_L:
				case NLoptAlgorithm.AUGLAG:
				case NLoptAlgorithm.AUGLAG_EQ:
				case NLoptAlgorithm.LN_AUGLAG:
				case NLoptAlgorithm.LN_AUGLAG_EQ:
				case NLoptAlgorithm.LD_AUGLAG:
				case NLoptAlgorithm.LD_AUGLAG_EQ:
					break;

				default:
					throw new ArgumentException("Algorithm " + algorithm.ToString() + " does not support inequality constraint.");
			}
		}

		/// <summary>
		/// Check if an equality constaint can be applied on current algorithm.
		/// In NLopt, api/options.c has a function equality_ok() which do the same verification.
		/// </summary>
		protected void CheckEqualityConstraintAvailability()
		{
			NLoptAlgorithm algorithm = Algorithm;
			switch (algorithm)
			{
				case NLoptAlgorithm.LD_SLSQP:
				case NLoptAlgorithm.GN_ISRES:
				case NLoptAlgorithm.LN_COBYLA:
				case NLoptAlgorithm.AUGLAG:
				case NLoptAlgorithm.AUGLAG_EQ:
				case NLoptAlgorithm.LN_AUGLAG:
				case NLoptAlgorithm.LN_AUGLAG_EQ:
				case NLoptAlgorithm.LD_AUGLAG:
				case NLoptAlgorithm.LD_AUGLAG_EQ:
					break;

				default:
					throw new ArgumentException("Algorithm " + algorithm.ToString() + " does not support equality constraint.");
			}
		}

		public void AddLessOrEqualZeroConstraint(Func<double[], double> constraint, double tolerance = 0.001)
		{
			CheckInequalityConstraintAvailability();
			nlopt_func func = (n, values, gradient, data) =>
			{
				CheckGradientHandling(gradient);
				return Evaluate((int)n, values, constraint);
			};
			_funcCache.Add((constraint, func));

			var res = nlopt_add_inequality_constraint(_opt, func, IntPtr.Zero, tolerance);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to add the constraint. Result: " + res, "constraint");
		}

		/// <summary>
		/// The gradient and current variables are passed to the constraint.
		/// </summary>
		public void AddLessOrEqualZeroConstraint(Func<double[], double[], double> constraint, double tolerance = 0.001)
		{
			CheckInequalityConstraintAvailability();
			nlopt_func func = (n, values, gradient, data) => Evaluate((int)n, values, gradient, constraint);
			_funcCache.Add((constraint, func));
			var res = nlopt_add_inequality_constraint(_opt, func, IntPtr.Zero, tolerance);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to add the constraint. Result: " + res, "constraint");
		}

		public void AddEqualZeroConstraint(Func<double[], double> constraint, double tolerance = 0.001)
		{
			CheckEqualityConstraintAvailability();
			nlopt_func func = (n, values, gradient, data) =>
			{
				CheckGradientHandling(gradient);
				return Evaluate((int)n, values, constraint);
			};
			_funcCache.Add((constraint, func));

			var res = nlopt_add_equality_constraint(_opt, func, IntPtr.Zero, tolerance);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to add the constraint. Result: " + res, "constraint");
		}

		/// <summary>
		/// The gradient and current variables are passed to the constraint.
		/// </summary>
		public void AddEqualZeroConstraint(Func<double[], double[], double> constraint, double tolerance = 0.001)
		{
			CheckEqualityConstraintAvailability();
			nlopt_func func = (n, values, gradient, data) => Evaluate((int)n, values, gradient, constraint);
			_funcCache.Add((constraint, func));
			var res = nlopt_add_equality_constraint(_opt, func, IntPtr.Zero, tolerance);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to add the constraint. Result: " + res, "constraint");
		}
		public void AddEqualZeroConstraints(Action<double[], double[]> constraints, double[] tolerances)
		{
			CheckEqualityConstraintAvailability();
			nlopt_mfunc mfunc = (m, results, n, values, gradient, data) => Evaluate((int)n, values, (int)m, results, constraints);
			_mfuncCache.Add(constraints, mfunc);

			var res = nlopt_add_equality_mconstraint(_opt, (uint)tolerances.Length, mfunc, IntPtr.Zero, tolerances);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to add the constraints. Result: " + res, "constraint");
		}

		public void AddEqualZeroConstraints(Action<double[], double[], double[]> constraints, double[] tolerances)
		{
			CheckEqualityConstraintAvailability();
			nlopt_mfunc mfunc = (m, results, n, values, gradient, data) => Evaluate((int)n, values, gradient, (int)m, results, constraints);
			_mfuncCache.Add(constraints, mfunc);

			var res = nlopt_add_equality_mconstraint(_opt, (uint)tolerances.Length, mfunc, IntPtr.Zero, tolerances);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to add the constraints. Result: " + res, "constraint");
		}

		public void AddLessOrEqualZeroConstraints(Action<double[], double[]> constraints, double[] tolerances)
		{
			CheckInequalityConstraintAvailability();
			nlopt_mfunc mfunc = (m, results, n, values, gradient, data) => Evaluate((int)n, values, (int)m, results, constraints);
			_mfuncCache.Add(constraints, mfunc);

			var res = nlopt_add_inequality_mconstraint(_opt, (uint)tolerances.Length, mfunc, IntPtr.Zero, tolerances);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to add the constraints. Result: " + res, "constraint");
		}

		public void AddLessOrEqualZeroConstraints(Action<double[], double[], double[]> constraints, double[] tolerances)
		{
			CheckInequalityConstraintAvailability();
			nlopt_mfunc mfunc = (m, results, n, values, gradient, data) => Evaluate((int)n, values, gradient, (int)m, results, constraints);
			_mfuncCache.Add(constraints, mfunc);

			var res = nlopt_add_inequality_mconstraint(_opt, (uint)tolerances.Length, mfunc, IntPtr.Zero, tolerances);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to add the constraints. Result: " + res, "constraint");
		}


		public void SetMinObjective(Func<double[], double> objective)
		{
			nlopt_func func = (n, values, gradient, data) => Evaluate((int)n, values, objective);
			_funcCache.Add((objective, func));
			var res = nlopt_set_min_objective(_opt, func, IntPtr.Zero);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the objective function. Result: " + res, "objective");
		}

		public void SetMinObjective(Func<double[], double[], double> objective)
		{
			nlopt_func func = (n, values, gradient, data) => Evaluate((int)n, values, gradient, objective);
			_funcCache.Add((objective, func));
			var res = nlopt_set_min_objective(_opt, func, IntPtr.Zero);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the objective function. Result: " + res, "objective");
		}

		public void SetMaxObjective(Func<double[], double> objective)
		{
			nlopt_func func = (n, values, gradient, data) => Evaluate((int)n, values, objective);
			_funcCache.Add((objective, func));
			var res = nlopt_set_max_objective(_opt, func, IntPtr.Zero);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the objective function. Result: " + res, "objective");
		}

		public void SetMaxObjective(Func<double[], double[], double> objective)
		{
			nlopt_func func = (n, values, gradient, data) => Evaluate((int)n, values, gradient, objective);
			_funcCache.Add((objective, func));
			var res = nlopt_set_max_objective(_opt, func, IntPtr.Zero);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the objective function. Result: " + res, "objective");
		}

		public void SetLowerBounds(params double[] minimums)
		{
			var res = nlopt_set_lower_bounds(_opt, minimums);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the lower bounds. Result: " + res, "minimums");
		}

		public void SetUpperBounds(params double[] maximums)
		{
			var res = nlopt_set_upper_bounds(_opt, maximums);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the upper bounds. Result: " + res, "maximums");
		}

		public NloptResult Optimize(double[] initialValues, out double? finishingObjectiveScore)
		{
			double temp = 0;
			var result = nlopt_optimize(_opt, initialValues, ref temp);
			finishingObjectiveScore = temp;
			return result;
		}

		public void SetInitialStepSize(double[] x)
		{
			var res = nlopt_set_initial_step(_opt, x);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the default initial step. Result: " + res);
		}

		public void SetInitialStepSize1(double x)
		{
			var res = nlopt_set_initial_step1(_opt, x);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the default initial step. Result: " + res);
		}

		public void ForceStop()
		{
			var res = nlopt_force_stop(_opt);
			if (res != NloptResult.FORCED_STOP && res < 0)
				throw new ArgumentException("Forced termination returned failure code " + res);
		}

		public void SetRelativeToleranceOnFunctionValue(double tol)
		{
			var res = nlopt_set_ftol_rel(_opt, tol);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the relative tolerance on function value. Result: " + res);
		}

		public void SetAbsoluteToleranceOnFunctionValue(double tol)
		{
			var res = nlopt_set_ftol_abs(_opt, tol);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the absolute tolerance on function value. Result: " + res);
		}

		public void SetRelativeToleranceOnOptimizationParameter(double tol)
		{
			var res = nlopt_set_xtol_rel(_opt, tol);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the relative tolerance on optimization parameter. Result: " + res);
		}

		public void SetAbsoluteToleranceOnOptimizationParameter(double tol)
		{
			var res = nlopt_set_xtol_abs1(_opt, tol);
			if (res != NloptResult.SUCCESS)
				throw new ArgumentException("Unable to set the absolute tolerance on optimization parameter. Result: " + res);
		}


		public double GetRelativeToleranceOnFunctionValue()
		{
			return nlopt_get_ftol_rel(_opt);
		}

		public double GetAbsoluteToleranceOnFunctionValue()
		{
			return nlopt_get_ftol_abs(_opt);
		}

		public double GetRelativeToleranceOnOptimizationParameter()
		{
			return nlopt_get_xtol_rel(_opt);
		}

#if MONO
		/// <summary>
		/// Wrapper around <paramref name="func"/> to init marshalled data if required.
		/// </summary>
		/// <param name="n">Size of <paramref name="p_values"/> array</param>
		/// <param name="p_values">Memory pointer to C++ values array.</param>
		/// <param name="func">C# function delegate that must be evaluated</param>
		/// <returns><paramref name="func"/> evaluation result</returns>
		private double Evaluate(int n, IntPtr p_values, Func<double[], double> func)
		{
			// values is marshalled as IN only (read only)
			var values = new double[n];

			if (p_values != IntPtr.Zero)
			{
				Marshal.Copy(p_values, values, 0, n);
			}

			return func.Invoke(values);
		}
#else
		private double Evaluate(int n, double[] values, Func<double[], double> func)
		{
			return func.Invoke(values);
		}
#endif

#if MONO
		/// <summary>
		/// Wrapper around <paramref name="func"/> to init marshalled data if required.
		/// </summary>
		/// <param name="n">Size of <paramref name="p_gradient"/> and <paramref name="p_values"/> arrays</param>
		/// <param name="p_values">Memory pointer to C++ values array.</param>
		/// <param name="p_gradient">Memory pointer to C++ gradient array.</param>
		/// <param name="func">C# function delegate that must be evaluated</param>
		/// <returns><paramref name="func"/> evaluation result</returns>
		private double Evaluate(int n, IntPtr p_values, IntPtr p_gradient, Func<double[], double[], double> func)
		{
			// gradient is IN/OUT (read and write)
			// values is marshalled as IN only (read only)
			var gradient = new double[n];
			var values = new double[n];

			if (p_values != IntPtr.Zero)
			{
				Marshal.Copy(p_values, values, 0, n);
			}
			if (p_gradient != IntPtr.Zero)
			{
				Marshal.Copy(p_gradient, gradient, 0, n);
			}

			var result = func.Invoke(values, gradient);

			// gradient has been altered
			// Update C++ handles with new values
			if (p_gradient != IntPtr.Zero)
			{
				Marshal.Copy(gradient, 0, p_gradient, n);
			}

			return result;
		}
#else
		private double Evaluate(int n, double[] values, double[] gradient, Func<double[], double[], double> func)
		{
			return func.Invoke(values, gradient);
		}
#endif

#if MONO
		/// <summary>
		/// Wrapper around <paramref name="action"/> to init marshalled data if required.
		/// </summary>
		/// <param name="n">Size of <paramref name="p_values"/> array</param>
		/// <param name="p_values">Memory pointer to C++ values array.</param>
		/// <param name="m">Size of <paramref name="p_results"/> array</param>
		/// <param name="p_results">Memory pointer to C++ results array.</param>
		/// <param name="action">C# action delegate that must be evaluated</param>
		private void Evaluate(int n, IntPtr p_values, int m, IntPtr p_results, Action<double[], double[]> action)
		{
			// results is IN/OUT (read and write)
			// values is marshalled as IN only (read only)
			var results = new double[m];
			var values = new double[n];

			if (p_values != IntPtr.Zero)
			{
				Marshal.Copy(p_values, values, 0, n);
			}

			action.Invoke(values, results);

			// results has been altered
			// Update C++ handle with new values
			if (p_results != IntPtr.Zero)
			{
				Marshal.Copy(results, 0, p_results, m);
			}
		}
#else
		private void Evaluate(int n, double[] values, int m, double[] results, Action<double[], double[]> action)
		{
			action.Invoke(values, results);
		}
#endif

#if MONO
		/// <summary>
		/// Wrapper around <paramref name="action"/> to init marshalled data if required.
		/// </summary>
		/// <param name="n">Size of <paramref name="p_gradient"/> and <paramref name="p_values"/> arrays</param>
		/// <param name="p_values">Memory pointer to C++ values array.</param>
		/// <param name="p_gradient">Memory pointer to C++ gradient array.</param>
		/// <param name="m">Size of <paramref name="p_results"/> array</param>
		/// <param name="p_results">Memory pointer to C++ results array.</param>
		/// <param name="action">C# action delegate that must be evaluated</param>
		/// <returns><paramref name="action"/> evaluation result</returns>
		private void Evaluate(int n, IntPtr p_values, IntPtr p_gradient, int m, IntPtr p_results, Action<double[], double[], double[]> action)
		{
			// results and gradient are IN/OUT (read and write)
			// values is marshalled as IN only (read only)
			var results = new double[m];
			var values = new double[n];
			var gradient = new double[n];

			if (p_values != IntPtr.Zero)
			{
				Marshal.Copy(p_values, values, 0, n);
			}
			if (p_gradient != IntPtr.Zero)
			{
				Marshal.Copy(p_gradient, gradient, 0, n);
			}

			action.Invoke(values, gradient, results);

			// gradient and results have been altered
			// Update C++ handles with new values
			if (p_gradient != IntPtr.Zero)
			{
				Marshal.Copy(gradient, 0, p_gradient, n);
			}
			if (p_results != IntPtr.Zero)
			{
				Marshal.Copy(results, 0, p_results, m);
			}
		}
#else
		private void Evaluate(int n, double[] values, double[] gradient, int m, double[] results, Action<double[], double[], double[]> action)
		{
			action.Invoke(values, gradient, results);
		}
#endif

#if MONO
		/// <summary>
		/// Force user to handle gradient in its gradient function is using a gradient based algorithm.
		/// </summary>
		/// <param name="p_gradient">Memory pointer to C++ gradient array.</param>
		private void CheckGradientHandling(IntPtr p_gradient)
		{
			if (p_gradient != IntPtr.Zero)
				throw new InvalidOperationException("Expected the constraint to handle the gradient.");
		}
#else
		private void CheckGradientHandling(double[] gradient)
		{
			if (gradient != null)
				throw new InvalidOperationException("Expected the constraint to handle the gradient.");
		}
#endif
	}
}
