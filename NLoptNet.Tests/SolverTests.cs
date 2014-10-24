using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Diagnostics;

namespace NLoptNet.Tests
{
	[TestClass]
	public class SolverTests
	{
		[TestMethod]
		public void TestBasicParabolaNoDerivative()
		{
			for (int i = 0; i <= (int)NLoptAlgorithm.GN_ESCH; i++)
			{
				var algorithm = (NLoptAlgorithm)i;
				var algStr = algorithm.ToString();
				if (algStr.Contains("AUGLAG") || algStr.Contains("MLSL")) 
					continue;
				if (algStr.Substring(0, 3).Contains("D_"))
					continue;
				var sw = System.Diagnostics.Stopwatch.StartNew();
				int count = 0;
				using (var solver = new NLoptSolver(algorithm, 1, 0.01, 2000))
				{
					solver.SetLowerBounds(new[] { -10.0 });
					solver.SetUpperBounds(new[] { 100.0 });
					solver.SetMinObjective(variables => {
						count++;
						return Math.Pow(variables[0] - 3.0, 2.0) + 4.0;
					});
					double? final;
					var data = new[] { 2.0 };
					var result = solver.Optimize(data, out final);
					//Assert.AreEqual(NloptResult.XTOL_REACHED, result);
					//if (result == NloptResult.MAXEVAL_REACHED || result == NloptResult.XTOL_REACHED)
						//Assert.AreEqual(4.0, final.Value, 0.1);
					//	Assert.AreEqual(3.0, data[0], 0.01);
					Trace.WriteLine(string.Format("D:{0:F3}, R:{1:F3}, A:{2}, {3}", data[0], final.GetValueOrDefault(-1), algorithm, result));
				}
				Trace.WriteLine("Elapsed: " + sw.ElapsedMilliseconds + "ms, Iterations: " + count);
			}
		}

		[TestMethod]
		public void TestBasicParabola()
		{
			for (int i = 0; i <= (int)NLoptAlgorithm.GN_ESCH; i++)
			{
				var algorithm = (NLoptAlgorithm)i;
				var algStr = algorithm.ToString();
				if (algStr.Contains("AUGLAG") || algStr.Contains("MLSL"))
					continue;
				var sw = System.Diagnostics.Stopwatch.StartNew();
				int count = 0;
				using (var solver = new NLoptSolver(algorithm, 1, 0.0001, 2000))
				{
					solver.SetLowerBounds(new[] { -10.0 });
					solver.SetUpperBounds(new[] { 100.0 });
					solver.SetMinObjective((variables,gradient) =>
					{
						count++;
						if (gradient != null)
							gradient[0] = (variables[0] - 3.0) * 2.0;
						return Math.Pow(variables[0] - 3.0, 2.0) + 4.0;
					});
					double? final;
					var data = new[] { 2.0 };
					var result = solver.Optimize(data, out final);
					//Assert.AreEqual(NloptResult.XTOL_REACHED, result);
					//if (result == NloptResult.MAXEVAL_REACHED || result == NloptResult.XTOL_REACHED)
					//Assert.AreEqual(4.0, final.Value, 0.1);
					//	Assert.AreEqual(3.0, data[0], 0.01);
					Trace.WriteLine(string.Format("D:{0:F3}, R:{1:F3}, A:{2}, {3}", data[0], final.GetValueOrDefault(-1), algorithm, result));
				}
				Trace.WriteLine("Elapsed: " + sw.ElapsedMilliseconds + "ms, Iterations: " + count);
			}
		}

[TestMethod]
public void FindParabolaMinimum()
{
	using (var solver = new NLoptSolver(NLoptAlgorithm.LN_COBYLA, 1, 0.001, 100))
	{
		solver.SetLowerBounds(new[] { -10.0 });
		solver.SetUpperBounds(new[] { 100.0 });
				
		solver.SetMinObjective(variables =>
		{
			return Math.Pow(variables[0] - 3.0, 2.0) + 4.0;
		});
		double? finalScore;
		var initialValue = new[] { 2.0 };
		var result = solver.Optimize(initialValue, out finalScore);

		Assert.AreEqual(NloptResult.XTOL_REACHED, result);
		Assert.AreEqual(3.0, initialValue[0], 0.1);
		Assert.AreEqual(4.0, finalScore.Value, 0.1);
	}
}

		[TestMethod]
		public void FindParabolaMinimumWithDerivative()
		{
			using (var solver = new NLoptSolver(NLoptAlgorithm.LD_AUGLAG, 1, 0.01, 100, NLoptAlgorithm.LN_NELDERMEAD))
			{
				solver.SetLowerBounds(new[] { -10.0 });
				solver.SetUpperBounds(new[] { 100.0 });
				solver.AddLessOrEqualZeroConstraint((variables, gradient) =>
				{
					if (gradient != null)
						gradient[0] = 1.0;
					return variables[0] - 100.0;
				}); 
				solver.SetMinObjective((variables, gradient) =>
				{
					if (gradient != null)
						gradient[0] = (variables[0] - 3.0) * 2.0;
					return Math.Pow(variables[0] - 3.0, 2.0) + 4.0;
				});
				double? finalScore;
				var initialValue = new[] { 2.0 };
				var result = solver.Optimize(initialValue, out finalScore);

				Assert.AreEqual(3.0, initialValue[0], 0.01);
				Assert.AreEqual(4.0, finalScore.Value, 0.01);
			}
		}
	}
}
