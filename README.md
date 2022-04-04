NLoptNet
========

This is a C# wrapper around the NLopt C library. It includes both 32 and 64-bit DLLs for NLopt 2.6.1 (64-bit only on Linux). It inherits NLopt's LGPL license.
Thanks to [ASI](http://asirobots.com) for sponsoring some time on this project.

Example:
```csharp
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
```
You can use the derivative-based optimizers like this:
```csharp
solver.SetMinObjective((variables, gradient) =>
{
	if (gradient != null)
		gradient[0] = (variables[0] - 3.0) * 2.0;
	return Math.Pow(variables[0] - 3.0, 2.0) + 4.0;
});
```
And add constraints like this:
```csharp
solver.AddEqualZeroConstraint((variables, gradient) =>
{
	if (gradient != null)
		gradient[0] = 1.0;
	return variables[0] - 100.0;
});
```
To use the Augmented Lagrangian:
```csharp
using(var solver = new NLoptSolver(NLoptAlgorithm.LN_AUGLAG, 2, 0.001, 1000, NLoptAlgorithm.LN_SBPLX))
{
	solver.SetLowerBounds(new[] { 1.0, 1.0 });
	solver.SetUpperBounds(new[] { 200.0, 200.0 });
	solver.AddLessOrEqualZeroConstraint(Constrain);
	solver.SetMinObjective(Score);
	var result = solver.Optimize(best, out finalScore);
}
```
Contributions are welcome. Please read through the NLopt documentation before posting questions/issues here.
