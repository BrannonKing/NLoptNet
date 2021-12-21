namespace NLoptNet
{
    using System;
    using System.Runtime.InteropServices;

    public partial class NLoptSolver
    {
        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern void nlopt_version(out int major, out int minor, out int bugfix);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr nlopt_create(NLoptAlgorithm algorithm, uint n);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern void nlopt_destroy(IntPtr opt);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl, SetLastError = true)]
        private static extern NloptResult nlopt_optimize(IntPtr opt, double[] x, ref double result);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_min_objective(IntPtr opt, nlopt_func f, IntPtr data);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_max_objective(IntPtr opt, nlopt_func f, IntPtr data);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NLoptAlgorithm nlopt_get_algorithm(IntPtr opt);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern uint nlopt_get_dimension(IntPtr opt);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_lower_bounds(IntPtr opt, double[] lowerBounds);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_upper_bounds(IntPtr opt, double[] upperBounds);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_add_inequality_constraint(IntPtr opt, nlopt_func fc, IntPtr data,
            double tolerance);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_add_equality_constraint(IntPtr opt, nlopt_func fc, IntPtr data,
            double tolerance);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_add_equality_mconstraint(IntPtr opt, uint m, nlopt_mfunc fc,
            IntPtr data, double[] tolerances);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_add_inequality_mconstraint(IntPtr opt, uint m, nlopt_mfunc fc,
            IntPtr data, double[] tolerances);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_xtol_rel(IntPtr opt, double tolerance);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_local_optimizer(IntPtr opt, IntPtr local);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern void nlopt_set_maxeval(IntPtr opt, int maxeval);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_force_stop(IntPtr opt);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_initial_step(IntPtr opt, double[] dx);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_initial_step1(IntPtr opt, double dx);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_ftol_rel(IntPtr opt, double tol);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_ftol_abs(IntPtr opt, double tol);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_xtol_abs1(IntPtr opt, double tol);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_set_xtol_abs(IntPtr opt, double[] tol);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern double nlopt_get_ftol_rel(IntPtr opt);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern double nlopt_get_ftol_abs(IntPtr opt);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern double nlopt_get_xtol_rel(IntPtr opt);

        [DllImport("nlopt", CallingConvention = CallingConvention.Cdecl)]
        private static extern NloptResult nlopt_get_xtol_abs(IntPtr opt, out double[] tol);
    }
}