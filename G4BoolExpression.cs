using g4;
using System;
using System.Collections.Generic;

namespace G4Library
{
    public class SDFBoolean
    {
        public enum SDFBooleanType
        {
            Union,
            Intersection,
            Difference,
            //SmoothUnion,
            //SmoothDifference,
            //SmoothIntersection,
            Blend
        }

        // Union
        public static BoundedImplicitFunction3d CreateUnion(BoundedImplicitFunction3d a, BoundedImplicitFunction3d b)
        {
            return new ImplicitUnion3d { A = a, B = b };
        }

        // Difference
        public static BoundedImplicitFunction3d CreateDifference(BoundedImplicitFunction3d a, BoundedImplicitFunction3d b)
        {
            return new ImplicitDifference3d { A = a, B = b };
        }

        // Intersection
        public static BoundedImplicitFunction3d CreateIntersection(BoundedImplicitFunction3d a, BoundedImplicitFunction3d b)
        {
            return new ImplicitIntersection3d { A = a, B = b };
        }

        // Smooth Intersection
        private static BoundedImplicitFunction3d CreateSmoothIntersection(BoundedImplicitFunction3d a, BoundedImplicitFunction3d b)
        {
            return new ImplicitSmoothIntersection3d { A = a, B = b };
        }

        // Smooth Difference
        private static BoundedImplicitFunction3d CreateSmoothDifference(BoundedImplicitFunction3d a, BoundedImplicitFunction3d b)
        {
            return new ImplicitSmoothDifference3d { A = a, B = b };
        }

        // Smooth Union
        private static BoundedImplicitFunction3d CreateSmoothUnion(BoundedImplicitFunction3d a, BoundedImplicitFunction3d b)
        {
            return new ImplicitSmoothUnion3d { A = a, B = b };
        }

        // Blending
        public static BoundedImplicitFunction3d CreateBlend(BoundedImplicitFunction3d a, BoundedImplicitFunction3d b, double blend)
        {
            return new ImplicitBlend3d { A = a, B = b, Blend = blend };
        }
    }

    public class SDFExpressionBoolean
    {
        private BoundedImplicitFunction3d dmesh1;
        private BoundedImplicitFunction3d dmesh2;
        private SDFBoolean.SDFBooleanType operation;
        private double blendFactor;

        public SDFExpressionBoolean(BoundedImplicitFunction3d dmesh1, BoundedImplicitFunction3d dmesh2, SDFBoolean.SDFBooleanType operation, double blendFactor = 0)
        {
            this.dmesh1 = dmesh1;
            this.dmesh2 = dmesh2;
            this.operation = operation;
            this.blendFactor = blendFactor;
        }

        public BoundedImplicitFunction3d CreateResult()
        {
            return operation switch
            {
                SDFBoolean.SDFBooleanType.Union => SDFBoolean.CreateUnion(dmesh1, dmesh2),
                SDFBoolean.SDFBooleanType.Intersection => SDFBoolean.CreateIntersection(dmesh1, dmesh2),
                SDFBoolean.SDFBooleanType.Difference => SDFBoolean.CreateDifference(dmesh1, dmesh2),
                //SDFBoolean.SDFBooleanType.SmoothUnion => SDFBoolean.CreateSmoothUnion(dmesh1, dmesh2),
                //SDFBoolean.SDFBooleanType.SmoothDifference => SDFBoolean.CreateSmoothDifference(dmesh1, dmesh2),
                //SDFBoolean.SDFBooleanType.SmoothIntersection => SDFBoolean.CreateSmoothIntersection(dmesh1, dmesh2),
                SDFBoolean.SDFBooleanType.Blend => SDFBoolean.CreateBlend(dmesh1, dmesh2, blendFactor),
                _ => throw new InvalidOperationException("Unsupported operation type")
            };
        }
    }

    public static class ExpressionSDFParser
    {
        // Convert infix to postfix for expressions
        public static string InfixToPostfix(string expression)
        {
            var precedence = new Dictionary<char, int> { { '+', 1 }, { '-', 1 }, { '*', 2 }, { '/', 2 } };
            Stack<char> stack = new Stack<char>();
            string postfix = "";

            foreach (char token in expression)
            {
                if (char.IsLetter(token))
                {
                    postfix += token;
                }
                else if (precedence.ContainsKey(token))
                {
                    while (stack.Count > 0 && precedence.ContainsKey(stack.Peek()) && precedence[stack.Peek()] >= precedence[token])
                    {
                        postfix += stack.Pop();
                    }
                    stack.Push(token);
                }
            }

            while (stack.Count > 0)
            {
                postfix += stack.Pop();
            }

            return postfix;
        }


        public static BoundedImplicitFunction3d Parse(string expression, Dictionary<char, BoundedImplicitFunction3d> sdfMapping, double blendFactor = 0)
        {
            string postfix = InfixToPostfix(expression);
            Stack<BoundedImplicitFunction3d> stack = new Stack<BoundedImplicitFunction3d>();

            foreach (char token in postfix)
            {
                if (sdfMapping.ContainsKey(token))
                {
                    stack.Push(sdfMapping[token]);
                }
                else
                {
                    if (stack.Count < 2)
                    {
                        throw new InvalidOperationException("Invalid expression: insufficient BoundedImplicitFunction3d objects for operation.");
                    }

                    var dmesh2 = stack.Pop();
                    var dmesh1 = stack.Pop();

                    SDFBoolean.SDFBooleanType operation = token switch
                    {
                        '+' => SDFBoolean.SDFBooleanType.Union,
                        '-' => SDFBoolean.SDFBooleanType.Difference,
                        '/' => SDFBoolean.SDFBooleanType.Intersection,
                        '*' => SDFBoolean.SDFBooleanType.Blend,
                        _ => throw new InvalidOperationException("Unknown operator")
                    };

                    var expressionG4SDF = new SDFExpressionBoolean(dmesh1, dmesh2, operation, blendFactor);
                    stack.Push(expressionG4SDF.CreateResult());
                }
            }

            if (stack.Count != 1)
            {
                throw new InvalidOperationException("Invalid expression: unbalanced operations.");
            }

            return stack.Pop();
        }
    }
}
