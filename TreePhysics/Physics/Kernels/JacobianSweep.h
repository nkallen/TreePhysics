//###########################################################
// Compute the Givens angle (and half-angle)
//###########################################################

float sh = A01*0.5;
bool b = sh*sh >= Tiny_Number;
sh = b ? sh : 0.f;
float diff = A00-A11;
float ch = b ? diff : 1.f;

float sh2 = sh*sh, ch2 = ch*ch;
b = ch2 <= Four_Gamma_Squared * sh2;
float w = fast::rsqrt(sh2 + ch2);

sh = b ? Sine_Pi_Over_Eight : w*sh;
ch = b ? Cosine_Pi_Over_Eight : w*ch;

sh2 = sh*sh; ch2 = ch*ch;

float c = ch2-sh2;
float s = ch*sh;
s = s + s;

//###########################################################
// Perform the actual Givens conjugation
//###########################################################

float sh2_ch2 = sh2+ch2;
A22 = A22 * sh2_ch2;
A02 = A02 * sh2_ch2;
A12 = A12 * sh2_ch2;
A22 = A22 * sh2_ch2;

float tmp1 = s*A02;
float tmp2 = s*A12;
A02 = c*A02;
A12 = c*A12;
A02 = tmp2 + A02;
A12 = A12 - tmp1;

tmp2 = s*s;
tmp1 = A11 * tmp2;
float tmp3 = A00 * tmp2;
float tmp4 = c*c;
A00 = A00 * tmp4;
A11 = A11 * tmp4;
A00 = A00 + tmp1;
A11 = A11 + tmp3;
tmp4 = tmp4 - tmp2;
tmp2 = A01+A01;
A01 = A01 * tmp4;
tmp4 = c*s;
tmp2 = tmp2 * tmp4;
float tmp5 = diff * tmp4;
A00 = A00 + tmp2;
A01 = A01 - tmp5;
A11 = A11 - tmp2;

//###########################################################
// Compute the cumulative rotation, in quaternion form
//###########################################################

quatf tmp = sh * q;

q = ch * q;
QIZ = QIZ + tmp.w;
q.w = q.w - TMP3;
QIX = QIX + TMP2;
QIY = QIY - TMP1;
