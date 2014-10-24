namespace NLoptNet
{
	/// <summary>
	/// {G/L}{D/N}_* = global/local derivative/no-derivative optimization, respectively.
	/// *_RAND algorithms involve some randomization.
	/// _NOSCAL algorithms are *not* scaled to a unit hypercube (i.e. they are sensitive to the units of x)
	/// </summary>
	public enum NLoptAlgorithm
	{
		GN_DIRECT = 0,
		GN_DIRECT_L,
		GN_DIRECT_L_RAND,
		GN_DIRECT_NOSCAL,
		GN_DIRECT_L_NOSCAL,
		GN_DIRECT_L_RAND_NOSCAL,

		GN_ORIG_DIRECT,
		GN_ORIG_DIRECT_L,

		GD_STOGO,
		GD_STOGO_RAND,

		LD_LBFGS_NOCEDAL,

		LD_LBFGS,

		LN_PRAXIS,

		LD_VAR1,
		LD_VAR2,

		LD_TNEWTON,
		LD_TNEWTON_RESTART,
		LD_TNEWTON_PRECOND,
		LD_TNEWTON_PRECOND_RESTART,

		GN_CRS2_LM,

		GN_MLSL,
		GD_MLSL,
		GN_MLSL_LDS,
		GD_MLSL_LDS,

		LD_MMA,

		LN_COBYLA,

		LN_NEWUOA,
		LN_NEWUOA_BOUND,

		LN_NELDERMEAD,
		LN_SBPLX,

		LN_AUGLAG,
		LD_AUGLAG,
		LN_AUGLAG_EQ,
		LD_AUGLAG_EQ,

		LN_BOBYQA,

		GN_ISRES,

		// AUGLAG: new variants that require local_optimizer to be set, not with older constants for backwards compatibility
		AUGLAG,
		AUGLAG_EQ,

		G_MLSL,
		G_MLSL_LDS,

		LD_SLSQP,

		LD_CCSAQ,

		GN_ESCH,
	}
}
