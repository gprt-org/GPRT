// GAMMA = 3 + sqrt(8)
// C_STAR = cos(pi/8)
// S_STAR = sin(pi/8)

#define GAMMA 5.8284271247
#define C_STAR 0.9238795325
#define S_STAR 0.3826834323
#define SVD_EPS 0.0000001

struct QR_mats
{
    float3x3 Q;
    float3x3 R;
};

struct SVD_mats
{
    float3x3 U;
    float3x3 Sigma;
    float3x3 V;
};

float2 approx_givens_quat(float s_pp, float s_pq, float s_qq)
{
    float c_h = 2.0f * (s_pp - s_qq);
    float s_h2 = s_pq * s_pq;
    float c_h2 = c_h * c_h;
    if ((GAMMA * s_h2) < c_h2)
    {
        float omega = 1.0f / sqrt(s_h2 + c_h2);
        return float2(omega * c_h, omega * s_pq);
    }
    return float2(C_STAR, S_STAR);
}

// the quaternion is stored in vec4 like so:
// (c, s * vec3) meaning that .x = c
float3x3 quat_to_mat3(float4 quat)
{
    float qx2 = quat.y * quat.y;
    float qy2 = quat.z * quat.z;
    float qz2 = quat.w * quat.w;
    float qwqx = quat.x * quat.y;
    float qwqy = quat.x * quat.z;
    float qwqz = quat.x * quat.w;
    float qxqy = quat.y * quat.z;
    float qxqz = quat.y * quat.w;
    float qyqz = quat.z * quat.w;
    return float3x3(float3(1.0f - (2.0f * (qy2 + qz2)), 2.0f * (qxqy + qwqz), 2.0f * (qxqz - qwqy)), 
                    float3(2.0f * (qxqy - qwqz), 1.0f - (2.0f * (qx2 + qz2)), 2.0f * (qyqz + qwqx)), 
                    float3(2.0f * (qxqz + qwqy), 2.0f * (qyqz - qwqx), 1.0f - (2.0f * (qx2 + qy2))));
}

float3x3 symmetric_eigenanalysis(float3x3 A)
{
    float3x3 S = mul(A, transpose(A));
    float3x3 q = float3x3(float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), float3(0.0f, 0.0f, 1.0f));
    for (int i = 0; i < 5; i++)
    {
        float2 ch_sh = approx_givens_quat(S[0].x, S[0].y, S[1].y);
        float4 ch_sh_quat = float4(ch_sh.x, 0.0f, 0.0f, ch_sh.y);
        float3x3 q_mat = quat_to_mat3(ch_sh_quat);
        S = mul(q_mat, mul(S, transpose(q_mat)));
        q = mul(q_mat, q);
        
        ch_sh = approx_givens_quat(S[0].x, S[0].z, S[2].z);
        ch_sh_quat = float4(ch_sh.x, 0.0f, -ch_sh.y, 0.0f);
        q_mat = quat_to_mat3(ch_sh_quat);
        S = mul(q_mat, mul(S, transpose(q_mat)));
        q = mul(q_mat, q);
        
        ch_sh = approx_givens_quat(S[1].y, S[1].z, S[2].z);
        ch_sh_quat = float4(ch_sh.x, ch_sh.y, 0.0f, 0.0f);
        q_mat = quat_to_mat3(ch_sh_quat);
        S = mul(q_mat, mul(S, transpose(q_mat)));
        q = mul(q_mat, q);
    }
    return q;
}

float2 approx_qr_givens_quat(float a0, float a1)
{
    float rho = sqrt((a0 * a0) + (a1 * a1));
    float s_h = a1;
    float max_rho_eps = rho;
    if (rho <= SVD_EPS)
    {
        s_h = 0.0f;
        max_rho_eps = SVD_EPS;
    }
    float c_h = max_rho_eps + a0;
    if (a0 < 0.0f)
    {
        float temp = c_h - (2.0f * a0);
        c_h = s_h;
        s_h = temp;
    }
    float omega = 1.0f / sqrt((c_h * c_h) + (s_h * s_h));
    return float2(omega * c_h, omega * s_h);
}

QR_mats qr_decomp(float3x3 B)
{
    QR_mats qr_decomp_result;
    float3x3 R;
    // 1 0
    // (ch, 0, 0, sh)
    float2 ch_sh10 = approx_qr_givens_quat(B[0].x, B[0].y);
    float3x3 Q10 = quat_to_mat3(float4(ch_sh10.x, 0.0f, 0.0f, ch_sh10.y));
    R = mul(B, transpose(Q10));

    // 2 0
    // (ch, 0, -sh, 0)
    float2 ch_sh20 = approx_qr_givens_quat(R[0].x, R[0].z);
    float3x3 Q20 = quat_to_mat3(float4(ch_sh20.x, 0.0f, -ch_sh20.y, 0.0f));
    R = mul(R, transpose(Q20));

    // 2 1
    // (ch, sh, 0, 0)
    float2 ch_sh21 = approx_qr_givens_quat(R[1].y, R[1].z);
    float3x3 Q21 = quat_to_mat3(float4(ch_sh21.x, ch_sh21.y, 0.0f, 0.0f));
    R = mul(R, transpose(Q21));

    qr_decomp_result.R = R;
    
    qr_decomp_result.Q = mul(Q21, mul(Q20, Q10));
    return qr_decomp_result;
}

SVD_mats svd(float3x3 A)
{
    SVD_mats svd_result;
    
    svd_result.V = symmetric_eigenanalysis(A);

    float3x3 B = mul(svd_result.V, A);

    // sort singular values
    float rho0 = dot(B[0], B[0]);
    float rho1 = dot(B[1], B[1]);
    float rho2 = dot(B[2], B[2]);
    if (rho0 < rho1)
    {
        float3 temp = B[1];
        B[1] = -B[0];
        B[0] = temp;
        temp = svd_result.V[1];
        svd_result.V[1] = -svd_result.V[0];
        svd_result.V[0] = temp;
        float temp_rho = rho0;
        rho0 = rho1;
        rho1 = temp_rho;
    }
    if (rho0 < rho2)
    {
        float3 temp_1 = B[2];
        B[2] = -B[0];
        B[0] = temp_1;
        temp_1 = svd_result.V[2];
        svd_result.V[2] = -svd_result.V[0];
        svd_result.V[0] = temp_1;
        rho2 = rho0;
    }
    if (rho1 < rho2)
    {
        float3 temp_2 = B[2];
        B[2] = -B[1];
        B[1] = temp_2;
        temp_2 = svd_result.V[2];
        svd_result.V[2] = -svd_result.V[1];
        svd_result.V[1] = temp_2;
    }

    QR_mats QR = qr_decomp(B);
    svd_result.U = QR.Q;
    svd_result.Sigma = QR.R;
    return svd_result;
}

inline float3x3 inverse(float3x3 m)
{
    const float det = 
          m[0][0] * m[1][1] * m[2][2] - m[0][0] * m[1][2] * m[2][1]
        + m[0][1] * m[1][0] * m[2][2] - m[0][1] * m[1][2] * m[2][0]
        + m[0][2] * m[1][0] * m[2][1] - m[0][2] * m[1][1] * m[2][0];
    if (det == 0.0f)
    {
        return m;
    }
    else
    {
        const float idet = 1.0f / det;

        return float3x3(
            idet * (m[1][1] * m[2][2] - m[1][2] * m[2][1]),
            idet * (m[0][2] * m[2][1] - m[0][1] * m[2][2]),
            idet * (m[0][1] * m[1][2] - m[0][2] * m[1][1]),

            idet * (m[1][2] * m[2][0] - m[1][0] * m[2][2]),
            idet * (m[0][0] * m[2][2] - m[0][2] * m[2][0]),
            idet * (m[0][2] * m[1][0] - m[0][0] * m[1][2]),

            idet * (m[1][0] * m[2][1] - m[1][1] * m[2][0]),
            idet * (m[0][1] * m[2][0] - m[0][0] * m[2][1]),
            idet * (m[0][0] * m[1][1] - m[0][1] * m[1][0])
        );
    }
}

float3x3 cholesky_decomp(float3x3 A)
{
    float3x3 L = float3x3(0,0,0,0,0,0,0,0,0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j <= i; ++j) {
            int sum = 0;
            if (j == i) // summation for diagonals
            {
                for (int k = 0; k < j; k++)
                    sum += pow(L[j][k], 2);
                L[j][j] = sqrt(A[j][j] - sum);
            } else {
                // Evaluating L(i, j) using L(j, j)
                for (int k = 0; k < j; k++)
                    sum += (L[i][k] * L[j][k]);
                L[i][j] = (A[i][j] - sum) / L[j][j];
            }
        }
    }
    return L;
}


// Constants
#define M_SQRT3    1.73205081f   // sqrt(3)
#define FLT_EPSILON     1.192092896e-07f

// Macros
#define SQR(x)      ((x)*(x))                        // x^2 

// ----------------------------------------------------------------------------
void dsyevc3(float3x3 A, inout float3 w)
	// ----------------------------------------------------------------------------
	// Calculates the eigenvalues of a symmetric 3x3 matrix A using Cardano's
	// analytical algorithm.
	// Only the diagonal and upper triangular parts of A are accessed. The access
	// is read-only.
	// ----------------------------------------------------------------------------
	// Parameters:
	//   A: The symmetric input matrix
	//   w: Storage buffer for eigenvalues
	// ----------------------------------------------------------------------------
	// Return value:
	//   0: Success
	//  -1: Error
	// ----------------------------------------------------------------------------
{
	float m, c1, c0;

	// Determine coefficients of characteristic poynomial. We write
	//       | a   d   f  |
	//  A =  | d*  b   e  |
	//       | f*  e*  c  |
	float de = A[0][1] * A[1][2];                                    // d * e
	float dd = SQR(A[0][1]);                                         // d^2
	float ee = SQR(A[1][2]);                                         // e^2
	float ff = SQR(A[0][2]);                                         // f^2
	m  = A[0][0] + A[1][1] + A[2][2];
	c1 = (A[0][0]*A[1][1] + A[0][0]*A[2][2] + A[1][1]*A[2][2])        // a*b + a*c + b*c - d^2 - e^2 - f^2
		- (dd + ee + ff);
	c0 = A[2][2]*dd + A[0][0]*ee + A[1][1]*ff - A[0][0]*A[1][1]*A[2][2]
	- 2.0f * A[0][2]*de;                                     // c*d^2 + a*e^2 + b*f^2 - a*b*c - 2*f*d*e)

	float p, sqrt_p, q, c, s, phi;
	p = SQR(m) - 3.0f*c1;
	q = m*(p - (3.0f/2.0f)*c1) - (27.0f/2.0f)*c0;
	sqrt_p = sqrt(abs(p));

	phi = 27.0f * ( 0.25f*SQR(c1)*(p - c1) + c0*(q + 27.0f/4.0f*c0));
	phi = (1.0f/3.0f) * atan2(sqrt(abs(phi)), q);

	c = sqrt_p*cos(phi);
	s = (1.0f/M_SQRT3)*sqrt_p*sin(phi);

	w[1]  = (1.0f/3.0f)*(m - c);
	w[2]  = w[1] + s;
	w[0]  = w[1] + c;
	w[1] -= s;
}

// ----------------------------------------------------------------------------
void dsyevv3(float3x3 A, inout float3x3 Q, inout float3 w)
	// ----------------------------------------------------------------------------
	// Calculates the eigenvalues and normalized eigenvectors of a symmetric 3x3
	// matrix A using Cardano's method for the eigenvalues and an analytical
	// method based on vector cross products for the eigenvectors.
	// Only the diagonal and upper triangular parts of A need to contain meaningful
	// values. However, all of A may be used as temporary storage and may hence be
	// destroyed.
	// ----------------------------------------------------------------------------
	// Parameters:
	//   A: The symmetric input matrix
	//   Q: Storage buffer for eigenvectors
	//   w: Storage buffer for eigenvalues
	// ----------------------------------------------------------------------------
	// Return value:
	//   0: Success
	//  -1: Error
	// ----------------------------------------------------------------------------
	// Dependencies:
	//   dsyevc3()
	// ----------------------------------------------------------------------------
	// Version history:
	//   v1.1 (12 Mar 2012): Removed access to lower triangualr part of A
	//     (according to the documentation, only the upper triangular part needs
	//     to be filled)
	//   v1.0f: First released version
	// ----------------------------------------------------------------------------
{
#ifndef EVALS_ONLY
	float norm;          // Squared norm or inverse norm of current eigenvector
	float n0, n1;        // Norm of first and second columns of A
	float n0tmp, n1tmp;  // "Templates" for the calculation of n0/n1 - saves a few FLOPS
	float thresh;        // Small number used as threshold for floating point comparisons
	float error;         // Estimated maximum roundoff error in some steps
	float wmax;          // The eigenvalue of maximum modulus
	float f, t;          // Intermediate storage
	int i, j;             // Loop counters
#endif

	// Calculate eigenvalues
	dsyevc3(A, w);

#ifndef EVALS_ONLY
	wmax = abs(w[0]);
	if ((t=abs(w[1])) > wmax)
		wmax = t;
	if ((t=abs(w[2])) > wmax)
		wmax = t;
	thresh = SQR(8.0f * FLT_EPSILON * wmax);

	// Prepare calculation of eigenvectors
	n0tmp   = SQR(A[0][1]) + SQR(A[0][2]);
	n1tmp   = SQR(A[0][1]) + SQR(A[1][2]);
	Q[0][1] = A[0][1]*A[1][2] - A[0][2]*A[1][1];
	Q[1][1] = A[0][2]*A[0][1] - A[1][2]*A[0][0];
	Q[2][1] = SQR(A[0][1]);

	// Calculate first eigenvector by the formula
	//   v[0] = (A - w[0]).e1 x (A - w[0]).e2
	A[0][0] -= w[0];
	A[1][1] -= w[0];
	Q[0][0] = Q[0][1] + A[0][2]*w[0];
	Q[1][0] = Q[1][1] + A[1][2]*w[0];
	Q[2][0] = A[0][0]*A[1][1] - Q[2][1];
	norm    = SQR(Q[0][0]) + SQR(Q[1][0]) + SQR(Q[2][0]);
	n0      = n0tmp + SQR(A[0][0]);
	n1      = n1tmp + SQR(A[1][1]);
	error   = n0 * n1;

	if (n0 <= thresh)         // If the first column is zero, then (1,0,0) is an eigenvector
	{
		Q[0][0] = 1.0f;
		Q[1][0] = 0.0f;
		Q[2][0] = 0.0f;
	}
	else if (n1 <= thresh)    // If the second column is zero, then (0,1,0) is an eigenvector
	{
		Q[0][0] = 0.0f;
		Q[1][0] = 1.0f;
		Q[2][0] = 0.0f;
	}
	else if (norm < SQR(64.0f * FLT_EPSILON) * error)
	{                         // If angle between A[0] and A[1] is too small, don't use
		t = SQR(A[0][1]);       // cross product, but calculate v ~ (1, -A0/A1, 0)
		f = -A[0][0] / A[0][1];
		if (SQR(A[1][1]) > t)
		{
			t = SQR(A[1][1]);
			f = -A[0][1] / A[1][1];
		}
		if (SQR(A[1][2]) > t)
			f = -A[0][2] / A[1][2];
		norm    = 1.0f/sqrt(1 + SQR(f));
		Q[0][0] = norm;
		Q[1][0] = f * norm;
		Q[2][0] = 0.0f;
	}
	else                      // This is the standard branch
	{
		norm = sqrt(1.0f / norm);
		for (j=0; j < 3; j++)
			Q[j][0] = Q[j][0] * norm;
	}


	// Prepare calculation of second eigenvector
	t = w[0] - w[1];
	if (abs(t) > 8.0f * FLT_EPSILON * wmax)
	{
		// For non-degenerate eigenvalue, calculate second eigenvector by the formula
		//   v[1] = (A - w[1]).e1 x (A - w[1]).e2
		A[0][0] += t;
		A[1][1] += t;
		Q[0][1]  = Q[0][1] + A[0][2]*w[1];
		Q[1][1]  = Q[1][1] + A[1][2]*w[1];
		Q[2][1]  = A[0][0]*A[1][1] - Q[2][1];
		norm     = SQR(Q[0][1]) + SQR(Q[1][1]) + SQR(Q[2][1]);
		n0       = n0tmp + SQR(A[0][0]);
		n1       = n1tmp + SQR(A[1][1]);
		error    = n0 * n1;

		if (n0 <= thresh)       // If the first column is zero, then (1,0,0) is an eigenvector
		{
			Q[0][1] = 1.0f;
			Q[1][1] = 0.0f;
			Q[2][1] = 0.0f;
		}
		else if (n1 <= thresh)  // If the second column is zero, then (0,1,0) is an eigenvector
		{
			Q[0][1] = 0.0f;
			Q[1][1] = 1.0f;
			Q[2][1] = 0.0f;
		}
		else if (norm < SQR(64.0f * FLT_EPSILON) * error)
		{                       // If angle between A[0] and A[1] is too small, don't use
			t = SQR(A[0][1]);     // cross product, but calculate v ~ (1, -A0/A1, 0)
			f = -A[0][0] / A[0][1];
			if (SQR(A[1][1]) > t)
			{
				t = SQR(A[1][1]);
				f = -A[0][1] / A[1][1];
			}
			if (SQR(A[1][2]) > t)
				f = -A[0][2] / A[1][2];
			norm    = 1.0f/sqrt(1 + SQR(f));
			Q[0][1] = norm;
			Q[1][1] = f * norm;
			Q[2][1] = 0.0f;
		}
		else
		{
			norm = sqrt(1.0f / norm);
			for (j=0; j < 3; j++)
				Q[j][1] = Q[j][1] * norm;
		}
	}
	else
	{
		// For degenerate eigenvalue, calculate second eigenvector according to
		//   v[1] = v[0] x (A - w[1]).e[i]
		//   
		// This would really get to complicated if we could not assume all of A to
		// contain meaningful values.
		A[1][0]  = A[0][1];
		A[2][0]  = A[0][2];
		A[2][1]  = A[1][2];
		A[0][0] += w[0];
		A[1][1] += w[0];

		[unroll]
		for (i=0; i < 3; i++)
		{
			A[i][i] -= w[1];
			n0       = SQR(A[0][i]) + SQR(A[1][i]) + SQR(A[2][i]);
			if (n0 > thresh)
			{
				Q[0][1]  = Q[1][0]*A[2][i] - Q[2][0]*A[1][i];
				Q[1][1]  = Q[2][0]*A[0][i] - Q[0][0]*A[2][i];
				Q[2][1]  = Q[0][0]*A[1][i] - Q[1][0]*A[0][i];
				norm     = SQR(Q[0][1]) + SQR(Q[1][1]) + SQR(Q[2][1]);
				if (norm > SQR(256.0f * FLT_EPSILON) * n0) // Accept cross product only if the angle between
				{                                         // the two vectors was not too small
					norm = sqrt(1.0f / norm);
					for (j=0; j < 3; j++)
						Q[j][1] = Q[j][1] * norm;
					break;
				}
			}
		}

		if (i == 3)    // This means that any vector orthogonal to v[0] is an EV.
		{
			[unroll]
			for (j=0; j < 3; j++)
				if (Q[j][0] != 0.0f)                                   // Find nonzero element of v[0] ...
				{                                                     // ... and swap it with the next one
					norm          = 1.0f / sqrt(SQR(Q[j][0]) + SQR(Q[(j+1)%3][0]));
					Q[j][1]       = Q[(j+1)%3][0] * norm;
					Q[(j+1)%3][1] = -Q[j][0] * norm;
					Q[(j+2)%3][1] = 0.0f;
					break;
				}
		}
	}


	// Calculate third eigenvector according to
	//   v[2] = v[0] x v[1]
	Q[0][2] = Q[1][0]*Q[2][1] - Q[2][0]*Q[1][1];
	Q[1][2] = Q[2][0]*Q[0][1] - Q[0][0]*Q[2][1];
	Q[2][2] = Q[0][0]*Q[1][1] - Q[1][0]*Q[0][1];
#endif
}