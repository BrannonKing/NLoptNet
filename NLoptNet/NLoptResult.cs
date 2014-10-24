namespace NLoptNet
{
	public enum NloptResult
	{
		FAILURE = -1, /* generic failure code */
		INVALID_ARGS = -2,
		OUT_OF_MEMORY = -3,
		ROUNDOFF_LIMITED = -4,
		FORCED_STOP = -5,
		SUCCESS = 1, /* generic success code */
		STOPVAL_REACHED = 2,
		FTOL_REACHED = 3,
		XTOL_REACHED = 4,
		MAXEVAL_REACHED = 5,
		MAXTIME_REACHED = 6
	}
}
