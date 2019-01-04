package org.usfirst.frc.team3158.robot.splines;

public class SplineGenerator {

	public static CubicSpline GenerateSpline0(float x0, float x1, float x2, float y0, float y1, float y2)
    {
        float sigma1;

        float h0, h1; //diferencia de x
        float deltaY0, deltaY1; //diferencia de y

        // coeficientes del polinomio cubico del spline0, "a" es el coeficiente de x cubica, "b" el de x cuadrada, "c" el de x y "d" es el termino independiente
        float a0, b0, c0, d0;

        //Setea Deltas
        h0 = x1 - x0;
        h1 = x2 - x1;
        deltaY0 = y1 - y0;
        deltaY1 = y2 - y1;

        //Setea Sigma
        // La primera y segunda fraccion del numerador de la formula para Sigma1
        float fraccionA = deltaY1 / h1;
        float fraccionB = deltaY0 / h0;
        //Resta de fracciones del numerador de la formula de sigma1
        float restaFracciones = fraccionA - fraccionB;
        //suma de las h del denominador de la formula de sigma1
        float sumaH = h0 + h1;
        //Aplicacion de la formula de sigma1
        sigma1 = (6 * restaFracciones) / (2 * sumaH);


        // Todo esto se obtiene aplicando la formula del spline0, que interpola el punto 0 con el punto 1 (primer intervalo)
        a0 = sigma1 / (6 * h0);
        b0 = -((3 * sigma1 * x0) / (6 * h0));
        //Numerador del coeficiente de x de la formula para el spline0
        float numeradorC0 = (3 * sigma1 * x0 * x0) - (sigma1 * h0 * h0) - (6 * y0) + (6 * y1);
        c0 = numeradorC0 / (6 * h0);
        float numeradorD0 = (6 * y0 * x1) - (sigma1 * x0 * x0 * x0) + (sigma1 * h0 * h0 * x0) - (6 * y1 * x0);
        d0 = numeradorD0 / (6 * h0);

        return new CubicSpline(a0, b0, c0, d0);
    }
	
	public static CubicSpline GenerateSpline1(float x0, float x1, float x2, float y0, float y1, float y2)
	    {
	        float sigma1;

	        float h0, h1; //diferencia de x
	        float deltaY0, deltaY1; //diferencia de y

	        // coeficientes del polinomio cubico del spline0, "a" es el coeficiente de x cubica, "b" el de x cuadrada, "c" el de x y "d" es el termino independiente
	        float a1, b1, c1, d1;

	        //Setea Deltas
	        h0 = x1 - x0;
	        h1 = x2 - x1;
	        deltaY0 = y1 - y0;
	        deltaY1 = y2 - y1;

	        //Setea Sigma
	        // La primera y segunda fraccion del numerador de la formula para Sigma1
	        float fraccionA = deltaY1 / h1;
	        float fraccionB = deltaY0 / h0;
	        //Resta de fracciones del numerador de la formula de sigma1
	        float restaFracciones = fraccionA - fraccionB;
	        //suma de las h del denominador de la formula de sigma1
	        float sumaH = h0 + h1;
	        //Aplicacion de la formula de sigma1
	        sigma1 = (6 * restaFracciones) / (2 * sumaH);

	        //Aplica formula del spline 1
	        a1 = -(sigma1 / (6 * h1));
	        b1 = (3 * sigma1 * x2) / (6 * h1);
	        float numeradorC1 = (sigma1 * h1 * h1) + (6 * y2) - (6 * y1) - (3 * sigma1 * x2 * x2);
	        c1 = numeradorC1 / (6 * h1);
	        float numeradorD1 = (sigma1 * x2 * x2 * x2) - (sigma1 * h1 * h1 * x2) + (6 * y1 * x2) - (6 * y2 * x1);
	        d1 = numeradorD1 / (6 * h1);

	        return new CubicSpline(a1, b1, c1, d1);
	    }

	public static Derivate DerivateSpline(CubicSpline spline)
	    {
	        
	        // Coeficiente x cuadrada de la derivada del spline0
	        float p = (3 * spline.cubicX);
	        //Coeficiente x de la derivada del spline0
	        float q = (2 * spline.squareX);
	        //Termino independiente de la derivada del spline0
	        float r = spline.x;

	        return new Derivate(p, q, r);
	    }
}
