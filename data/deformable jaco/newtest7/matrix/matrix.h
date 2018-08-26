#ifndef __CPP_MATRIX_H__
#define __CPP_MATRIX_H__

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#define SNGLR_CHK // 特異値の確認

template <class T> class matrix;
template <class T> matrix<T> operator +(const matrix<T> &s1, const matrix<T> &s2);
template <class T> matrix<T> operator -(const matrix<T> &s1, const matrix<T> &s2);
template <class T> matrix<T> operator *(const matrix<T> &s1, const matrix<T> &s2);
template <class T> matrix<T> operator *(T a, const matrix<T> &s);
template <class T> matrix<T> operator *(const matrix<T> &s, T a);
template <class T> matrix<T> operator /(const matrix<T> &s, T a);
template <class T> matrix<T> operator |(const matrix<T> &s1, const matrix<T> &s2);
template <class T> matrix<T> operator ||(const matrix<T> &s1, const matrix<T> &s2);
template <class T> T operator &(const matrix<T> &s1, const matrix<T> &s2);	
template <class T> matrix<T> operator &&(const matrix<T> &s1, const matrix<T> &s2);
template <class T> void operator <<=(T *data, const matrix<T> &s);
template <class T> matrix<T> operator %(const matrix<T> &s1, const matrix<T> &s2);
template <class T> matrix<T> operator >(const matrix<T> &A, const matrix<T> &B);

template <class T> class matrix {
 private:
	//---- 行数，列数の確認が無いため，pubulic から private に変更 ----//
	// i行をaで割る
    void divi (int i, T a);
	// i1行からi2行をa倍したものを引く
    void subi (int i1, int i2, T a);
	// i1行とi2行とを入れ替える
    void swapi (int i1, int i2);
	// dj列を削除した行列を返す
    matrix<T> dj (int dj) const;
	// di行dj列を削除した行列を返す
    matrix<T> dij (int di, int dj) const;
	// エラー処理
	void err (const char *, int, int, int, int) const;
	// 行列の要素にアクセス(行や列のチェック無し，先頭要素は (1,1))
	T& elm (int mi, int mj);                  
	
 public:
	// デフォルト・コンストラクタ
    matrix (void);
	// コンストラクタ
    matrix (int i, int j, const T *s = NULL);
	//  行列sのコピーコンストラクタ
    matrix (const matrix<T> &s);
	// デストラクタ
    ~matrix ();

	// 行列に-1をかけたものを返す
    matrix<T> operator -() const;
	// 行列sと等しい時に真を返す
    int operator == (const matrix<T> &s) const;
	// 行列sと等しくない時に真を返す
    int operator != (const matrix<T> &s) const;

	// 行列s1に行列s2を加えたものを返す
    friend matrix<T> operator +<> (const matrix<T> &s1, const matrix<T> &s2);
	// 行列s1から行列s2を引いたものを返す
    friend matrix<T> (::operator -<>) (const matrix<T> &s1, const matrix<T> &s2);
	// 行列s1と行列s2との積を返す
    friend matrix<T> operator *<> (const matrix<T> &s1, const matrix<T> &s2);
	// 単位行列のa倍と行列sをかけ，その行列を返す (行列sをa倍したものを返す)
    friend matrix<T> operator *<> (T a, const matrix<T> &s);
	// 行列sと単位行列のa倍をかけ，その行列を返す (行列sをa倍したものを返す)
    friend matrix<T> operator *<> (const matrix<T> &s, T a);
	// 行列sと単位行列の1/a倍をかけ，その行列を返す (行列sをaで割ったものを返す)
    friend matrix<T> operator /<> (const matrix<T> &s, T a);
	// 行列s1の右に行列s2をつないだ行列を返す (先頭要素は (1,1))
    friend matrix<T> operator |<>  (const matrix<T> &s1, const matrix<T> &s2);
	// 行列s1の下に行列s2をつないだ行列を返す
    friend matrix<T> operator ||<> (const matrix<T> &s1, const matrix<T> &s2);
	// s1とs2との内積を返す (N x 1 行列のみ有効)
    friend T         operator &<>  (const matrix<T> &s1, const matrix<T> &s2);	
	// s1とs2との外積を返す (3 x 1 行列のみ有効)
    friend matrix<T> operator &&<> (const matrix<T> &s1, const matrix<T> &s2);
	// 行列の値を配列に代入 (配列の長さに気を付けること)
    friend void      operator <<=<> (T *data, const matrix<T> &s);
	// s1とs2の各要素の積を要素とする行列を返す (N x 1 行列のみ有効)
    friend matrix<T> operator %<>  (const matrix<T> &s1, const matrix<T> &s2);
	// ガウスの消去法により A^{-1}*B を求める
    friend matrix<T> operator ><>  (const matrix<T> &A, const matrix<T> &B);
	
	// 行列sを代入
    matrix<T>& operator = (const matrix<T> &s);	
	// 配列dataを代入する
	matrix<T>& operator <<= (const T *data);
	// 単位行列のa倍を代入
    matrix<T>& operator = (T a);
	// 行列sを加える
    matrix<T>& operator += (const matrix<T> &s);
	// 行列sで引く
    matrix<T>& operator -= (const matrix<T> &s);
	// 行列sとの積にする
    matrix<T>& operator *= (const matrix<T> &s);
	// 行列をaでかける
    matrix<T>& operator *= (T a);
	// 行列をaで割る
	matrix<T>& operator /= (T a);

	// 行列のトレースを求める
    T tr (void) const;
	// 行列式を求める
    T det (void) const;
	// rankを求める
    int rank (void) const;

	// 転置行列を求める
    matrix<T> t (void) const;
	// 逆行列を求める (部分pivot，Gauss-Jordan法を使用)
    matrix<T> inverse (void) const;
	// 累乗法により実対称行列の固有ベクトルを求める
	matrix<T> powEig (void) const;
	// 累乗法により実対称行列の固有値と固有ベクトルを求める
	matrix<T> powEig (matrix<T> &c)  const;

	// 2次元行列の大きさを知る
	void msize (int *i1, int *i2) const;
	// 行列の要素にアクセス (先頭要素は (1,1))
	T& operator () (int mi, int mj);
	// 行列sのある部分小行列として得る (先頭要素は (1,1))
	matrix<T> part (int mi, int mj, int ni, int nj) const;
	// 行列のある部分に小行列sを代入する (先頭要素は (1,1))
	void pasg (int mi, int mj, const matrix<T> &s);
	// 行列sのある部分を小行列として得る (先頭要素は (1,1))
    matrix<int> size(void) const;
	// 行列が正則(0)か特異(1)か判定する
    int snglr (T limit = 0.01) const;
	// ユークリッドノルムを返す (N x 1 行列のみ有効)
	double norm (void) const;
	// 正規化したベクトルを返す (N x 1 行列のみ有効)
        matrix<T> normal (void) const;
	// キャスト (配列の長さに気を付けること)
	matrix<double> cast_d(void) const;
	matrix<int> cast_i(void) const;
	// 行列を標準出力に出力する
	void print (const char *style = " %12.4f", const char *msg = "\n") const;

 protected:
    int ii, jj;
    T *aij;
};

// デフォルト・コンストラクタ
template <class T>
matrix<T>::matrix (void)
{
    ii = jj = 0;
    aij = NULL;
}

// コンストラクタ
template <class T>
matrix<T>::matrix (int i, int j, const T *s)
{
    size_t size = i * j + 1;

    ii = i; jj= j;
    aij = new T[size];

    if(s)
        memcpy(aij, s, size * sizeof(T));
    else
        memset(aij, 0, size * sizeof(T));
}

//  行列sのコピーコンストラクタ
template <class T>
matrix<T>::matrix (const matrix<T> &s)
{
    size_t size = s.ii * s.jj + 1;

    ii = s.ii; jj = s.jj;
    aij = new T[size];

    memcpy(aij, s.aij, size * sizeof(T));
}

// デストラクタ
template <class T>
matrix<T>::~matrix ()
{
    delete[] aij;
}

// エラー処理
template <class T>
void matrix<T>::err (const char *str, int i, int j, int k, int l) const
{
    printf("matrix_err_stop !!! \n");    
    printf(str, i, j, k, l);
	printf("oo oo oo oo oo  oo oo  oo oo oo \n");    
	printf("oo oo oo infinite loop oo oo oo \n");    
	printf("oo oo oo oo oo  oo oo  oo oo oo \n");    
}

// 行列に-1をかけたものを返す
template <class T>
matrix<T> matrix<T>::operator -() const
{
    if(!ii || !jj)
        err("operator -() (%d, %d)\n", ii, jj, 0, 0);

    matrix<T> d(*this);

    for(int i = 0; i < d.ii * d.jj; i++)
        d.aij[i] = -d.aij[i];

    return d;
}

// 行列sと等しい時に真を返す
template <class T>
int matrix<T>::operator == (const matrix<T> &s) const
{
    if(!ii || !jj || ii != s.ii || jj != s.jj)
        err("operator == (const matrix<T> &s), (%d, %d), (%d, %d)\n", 
            ii, jj, s.ii, s.jj);

    for(int i = 0; i < ii * jj; i++)
        if(aij[i] != s.aij[i])
            return 0;

    return 1;
}

// 行列sと等しくない時に真を返す
template <class T>
int matrix<T>::operator != (const matrix<T> &s) const
{

    if(!ii || !jj  || ii != s.ii || jj != s.jj)
        err("operator != (const matrix<T> &s), (%d, %d), (%d, %d)\n", 
            ii, jj, s.ii, s.jj);

    for(int i = 0; i < ii * jj; i++)
        if(aij[i] != s.aij[i])
            return 1;

    return 0;
}

// 行列s1に行列s2を加えたものを返す
template <class T>
matrix<T> operator + (const matrix<T> &s1, const matrix<T> &s2)
{
    if(!s1.ii || !s1.jj || s1.ii != s2.ii || s1.jj != s2.jj)
        s1.err("operator + (%d, %d), (%d, %d)\n", s1.ii, s1.jj, s2.ii, s2.jj);
    matrix<T> d(s1);

    for(int i = 0; i < s1.ii * s1.jj; i++)
        d.aij[i] += s2.aij[i];

    return d;
}

// 行列s1から行列s2を引いたものを返す
template <class T>
matrix<T> operator - (const matrix<T> &s1, const matrix<T> &s2)
{
    if(!s1.ii || !s1.jj || s1.ii != s2.ii || s1.jj != s2.jj)
        s1.err("operator - (%d, %d), (%d, %d)\n", s1.ii, s1.jj, s2.ii, s2.jj);

    matrix<T> d(s1);

    for(int i = 0; i < s1.ii * s1.jj; i++)
        d.aij[i] -= s2.aij[i];

    return d;
}

// 行列s1と行列s2との積を返す
template <class T>
matrix<T> operator * (const matrix<T> &s1, const matrix<T> &s2)
{
    if(!s1.ii || !s1.jj || !s2.jj || s1.jj != s2.ii)
        s1.err(
            "operator * (matrix<T>, matrix<T>)(%d, %d), (%d, %d)\n", 
			s1.ii, s1.jj, s2.ii, s2.jj);

    matrix<T> d(s1.ii, s2.jj);

    for(int i = 0; i < s1.ii; i++)
        for(int j = 0; j < s2.jj; j++)
            for(int k = 0; k < s1.jj; k++)
                d.aij[i * d.jj + j] += 
					s1.aij[i * s1.jj + k] * s2.aij[k * s2.jj + j];

    return d;
}

// 単位行列のa倍と行列sをかけ，その行列を返す (行列sをa倍したものを返す)
template <class T>
matrix<T> operator * (T a, const matrix<T> &s)
{
	if(!s.ii || !s.jj)
        s.err("operator * (T a, const matrix<T> &s), (%d, %d), \n", s.ii, s.jj, 0, 0);
	matrix<T> d(s);

    for(int i = 0; i < s.ii * s.jj; i++)
        d.aij[i] *= a;

    return d;
}

// 行列sと単位行列のa倍をかけ，その行列を返す (行列sをa倍したものを返す)
template <class T>
matrix<T> operator * (const matrix<T> &s, T a)
{
    if(!s.ii || !s.jj)
        s.err("operator * (const matrix<T> &s, T a), (%d, %d), \n", s.ii, s.jj, 0, 0);

    matrix<T> d(s);

    for(int i = 0; i < s.ii * s.jj; i++)
        d.aij[i] *= a;
	
    return d;
}

// 行列sと単位行列の1/a倍をかけ，その行列を返す (行列sをaで割ったものを返す)
template <class T>
matrix<T> operator / (const matrix<T> &s, T a)
{
    if(!s.ii || !s.jj)
        s.err("operator / (const matrix<T> &s, T a), (%d, %d), \n", s.ii, s.jj, 0, 0);
    matrix<T> d(s);

    for(int i = 0; i < s.ii * s.jj; i++)
        d.aij[i] /= a;

    return d;
}

// 行列sを代入 (一部改造)
template <class T>
matrix<T>& matrix<T>::operator = (const matrix<T> &s) 
{
    if(!ii || !jj || ii != s.ii || jj != s.jj)	
        err("operator = (const matrix<T> &s), (%d, %d), (%d, %d)\n", ii, jj, s.ii, s.jj);
	
    size_t size = s.ii * s.jj;

    memcpy(aij, s.aij, size * sizeof(T));

    return *this;
}

// 配列dataを代入する
template <class T>
matrix<T>& matrix<T>::operator <<= (const T *data)
{
    if(!ii || !jj)	
        err("operator  = (const T *data),  (%d, %d), (%d, %d)\n", ii, jj, 0, 0);
	
    size_t size = ii * jj;
    memcpy(aij, data, size * sizeof(T));

    return *this;
}

// 単位行列のa倍を代入
template <class T>
matrix<T>& matrix<T>::operator = (T a)
{
    if(!ii || !jj)
        err("operator = (T a), (%d, %d)\n", ii, jj, 0, 0);
	
    for(int i = 0; i < ii; i++)
        for(int j = 0; j < jj; j++)
            if(i == j)
                aij[i * jj + j] = a;
            else
                aij[i * jj + j] = 0;
	
    return *this;
}

// 行列sを加える
template <class T>
matrix<T>& matrix<T>::operator += (const matrix<T> &s)
{
    if(!ii || !jj || ii != s.ii || jj != s.jj)
        err("operator += (const matrix<T> &s) (%d, %d), (%d, %d)\n", ii, jj, s.ii, s.jj);

    for(int i = 0; i < ii * jj; i++)
        aij[i] += s.aij[i];
	
    return *this;
}

// 行列sで引く
template <class T>
matrix<T>& matrix<T>::operator -= (const matrix<T> &s)
{
    if(!ii || !jj  || ii != s.ii || jj != s.jj)
        err("operator -= (const matrix<T> &s) (%d, %d), (%d, %d)\n", ii, jj, s.ii, s.jj);

    for(int i = 0; i < ii * jj; i++)
        aij[i] -= s.aij[i];

    return *this;
}

// 行列sとの積にする
template <class T>
matrix<T>& matrix<T>::operator *= (const matrix<T> &s)
{
    if(!ii || !jj  || ii != s.ii || jj != s.jj)
        err("operator *= (const matrix<T> &s) (%d, %d), (%d, %d)\n", ii, jj, s.ii, s.jj);
	
    matrix<T> d(ii, s.jj);
	
    for(int i = 0; i < ii; i++)
        for(int j = 0; j < s.jj; j++)
            for(int k = 0; k < jj; k++)
                d.aij[i * d.jj + j] +=
					aij[i * jj + k] * s.aij[k * s.jj + j];
	
    return *this = d;
}

// 行列をaでかける
template <class T>
matrix<T>& matrix<T>::operator *= (T a)
{
    if(!ii || !jj)
        err("operator *= (T a) (%d, %d)\n", ii, jj, 0, 0);

    for(int i = 0; i < ii * jj; i++)
        aij[i] *= a;

    return *this;
}

// 行列をaで割る
template <class T>
matrix<T>& matrix<T>::operator /= (T a)
{
    if(!ii || !jj)
        err("operator /= (T a) (%d, %d)\n", ii, jj, 0, 0);

    for(int i = 0; i < ii * jj; i++)
        aij[i] /= a;

    return *this;
}

// i行をaで割る
template <class T>
void matrix<T>::divi (int i, T a)
{
    for(int j = 0; j < jj; j++)
        aij[i * jj + j] /= a;
}

// i1行からi2行をa倍したものを引く
template <class T>
void matrix<T>::subi (int i1, int i2, T a)
{
    for(int j = 0; j < jj; j++)
        aij[i1 * jj + j] -= aij[i2 * jj + j] * a;
}

// i1行とi2行とを入れ替える
template <class T>
void matrix<T>::swapi (int i1, int i2)
{
    for(int j = 0; j < jj; j++){
        T t;
		
        t = aij[i1 * jj + j];
        aij[i1 * jj + j] = aij[i2 * jj + j];
        aij[i2 * jj + j] = t;
    }
}

// 2次元行列の大きさを知る
template <class T>
void matrix<T>::msize (int *i1, int *i2) const
{
	*i1 = ii;
	*i2 = jj;
}

// 行列のトレースを求める
template <class T>
T matrix<T>::tr (void) const
{
    if(!ii || ii != jj)
        err("tr (%d, %d)\n", ii, jj, 0, 0);

    T r = 0;

    if(ii != jj)
        return 0;

    for(int i = 0; i < ii; i++)
        r += aij[i * (jj + 1)];
	
    return r;
}

// 行列式を求める
template <class T>
T matrix<T>::det (void) const
{
    if(!ii || ii != jj)
        err("det (%d, %d)\n", ii, jj, 0, 0);

    int i;

    if(ii != jj)      // 行と列は同じであること
        return 0;

    switch (ii){
	case 1:        // スカラー
		return aij[0];
		break;
	case 2:        // 2×2行列
		return aij[0] * aij[3] - aij[1] * aij[2];
		break;
	default:        // それ以上
		matrix<T> d(*this);

		if(d.aij[0] == 0)  // (0,0)を0以外になるようにする
			for(i = 1; i < d.ii; i++)
				if(d.aij[i * d.jj]){
					d.swapi(0, i);    // 行の入れ替え
					d *= -1;
					break;
				}
			
		T a = d.aij[0];
		if(a == 0)
			return 0;        // どの行も0なら解は0
			
		// (0,0)以外の0列の値を0にするように行の引き算
		for(i = 1; i < d.ii; i++)
			d.subi(i, 0, d.aij[i * d.jj] / a);
			
		return a * d.dij(0, 0).det();
		break;
    }
    return 0;
}

// rankを求める
template <class T>
int matrix<T>::rank (void) const
{
    if(!ii || !jj)
        err("rank (%d, %d)\n", ii, jj, 0, 0);
	
    int i;
    matrix<T> d(*this);

    if(d.aij[0] == 0)
        for(i = 1; i < d.ii; i++)
            if(d.aij[i * d.jj]){
                d.swapi(0, i);
                break;
            }

    T a = d.aij[0];

    if(d.ii == 1 || d.jj == 1){
        if(a != 0)
            return 1;
        else
            return 0;
    }
	
    if(a != 0){
        for(i = 1; i < d.ii; i++)
            d.subi(i, 0, d.aij[i * d.jj] / a);
		
        return (1 + (d.dij(0, 0)).rank());
    }
	
    return d.dj(0).rank();
}

// 転置行列を求める
template <class T>
matrix<T> matrix<T>::t (void) const
{
    if(!ii || !jj)
        err("t (%d, %d)\n", ii, jj, 0, 0);
	
    matrix<T> d(jj, ii);

    for(int i = 0; i < ii; i++)
        for(int j = 0; j < jj; j++)
			d.aij[j * ii + i] = aij[i * jj + j];
	
    return d;
}

// dj列を削除した行列を返す
template <class T>
matrix<T> matrix<T>::dj (int dj) const
{
    matrix<T> d(ii, jj - 1);
	
    for(int i = 0, k = 0; i < ii; i++)
        for(int j = 0; j < jj; j++){
            if(j == dj) continue;
			
            d.aij[k++] = aij[i * jj + j];
        }
	
    return d;
}

// di行dj列を削除した行列を返す
template <class T>
matrix<T> matrix<T>::dij (int di, int dj) const
{
    matrix<T> d(ii - 1, jj - 1);
	
    for(int i = 0, k = 0; i < ii; i++){
        if(i == di) continue;
		
        for(int j = 0; j < jj; j++){
            if(j == dj) continue;
            d.aij[k++] = aij[i * jj + j];
        }
    }
	
    return d;
}

// 逆行列を求める (部分pivot，Gauss-Jordan法を使用)
template <class T>
matrix<T> matrix<T>::inverse (void) const
{
    if(!ii || ii != jj)
        err("inverse (%d, %d)\n", ii, jj, 0, 0);
	
    matrix<T> s(*this), d(ii, jj);
    T a, b;
	
    if(ii != jj || det() == 0)
        return d;
	
    d = 1;
    for(int j = 0; j < s.jj; j++){
        int i;
		
        // 部分pivot
        a = 0;
        int k = j;
        for(i = j; i < s.ii; i++)
            if(a < (b = fabs(s.aij[i * s.jj + j]))){
                k = i;
                a = b;
            }
        if(k != j){
            s.swapi(j, k);
            d.swapi(j, k);
        }

        //Gauss-Jordan法
        a = s.aij[j * (s.jj + 1)];
        for(i = 0; i < s.ii; i++){
            if(i == j) continue;
			
            b = s.aij[i * s.jj + j] / a;
            s.subi(i, j, b);
            d.subi(i, j, b);
        }
        s.divi(j, a);
        d.divi(j, a);
    }

    return d;
}

// 累乗法により実対称行列の固有値と固有ベクトルを求める
template <class T>
matrix<T> matrix<T>::powEig (matrix<T> &c) const
{
    if(!ii || ii != jj || ii != c.ii || c.ii != c.jj)
        err("powEig (%d, %d)\n", ii, jj, 0, 0);

    matrix<T> s(*this), d(ii, jj), x(ii, 1), y(ii, 1);
	double eps = 0.000001, min0 = 1.0e-20, max0 = 1.0e20, min, max, p, q;
	int it_max = 500;
	
    if(ii != jj)
        return d;
	
	for(int m = 0; m < jj; m++){
		// 出発値を設定
		for(int i = 0; i < ii; i++)
			x.aij[i] = 4096 - (rand() & 2047);
		x = x.normal();
		
		for(int i = 0; i < it_max; i++){
			// y = Ax の計算
			y = s * x;
			
			// 固有値計算
			min = max0;	max = - max0;
			for(int j = 0; j < ii; j++){
				if (fabs(y.aij[j]) < min0)
					min = min0;
				else{
					if(fabs(x.aij[j]) > min0){
						q = y.aij[j] / x.aij[j];
						
						if(q < min) min = q;
						if(q > max) max = q;
					}
					else
						max = max0;
				}
			}
			// 収束判定
			if((max - min) <= eps)
				break;
			// yをxにコピーして正規化
			x = y.normal();
		}
		// yをxにコピーして正規化
		x = y.normal();
		p = 0.0;
		
		for(int i = 0; i < ii; i++)
			for(int j = 0; j < jj; j++)
				p += aij[i * jj + j] * x.aij[i] * x.aij[j];
		
		// 固有値を対角成分に格納
		c.aij[m * c.jj + m] = p;
		
		// 固有ベクトルを格納
		for(int i = 0; i < ii; i++)
			d.aij[i * d.jj + m] = x.aij[i];
		
		// 減次作業
		for(int i = 0; i < ii; i++)
			for(int j = 0; j < jj; j++)
				s.aij[i * s.jj + j] -= p * x.aij[i] * x.aij[j];
	}
    return d;
}

// 累乗法により実対称行列の固有ベクトルを求める
template <class T>
matrix<T> matrix<T>::powEig (void) const
{
	if(!ii || ii != jj)
		err("powEig (%d, %d)\n", ii, jj, 0, 0);
	
	matrix<T> s(*this), c(ii, jj), d(ii, jj);
	
	if(ii != jj)
		return d;
	
	d = s.powEig(c);	
	return d;
}

// 行列を標準出力に出力する
template <class T>
void matrix<T>::print (const char *style, const char *msg) const
{
    if(!ii  || !jj)
        err("print (%d, %d)\n", ii, jj, 0, 0);

	printf(msg);	
    for(int i = 0; i < ii; i++){
        for(int j = 0; j < jj; j++)
            printf(style, aij[i * jj + j]);
        printf("\n");
    }
}

// 行列の要素にアクセス (先頭要素は (1,1))
template <class T>
inline T& matrix<T>::operator () (int mi, int mj)
{
	if(ii < mi-1 || jj < mj-1)
		err("operator () (const matrix<T> &s) (%d, %d), (%d, %d)\n", ii, jj, mi, mj);
	
	return aij[(mi-1)*jj + mj-1];
}	

// 行列の要素にアクセス(行や列のチェック無し，先頭要素は (1,1))
template <class T>
inline T& matrix<T>::elm (int mi, int mj)
{
	return aij[(mi-1)*jj + mj-1];
}	

// 行列sのある部分小行列として得る (先頭要素は (1,1))
template <class T>
matrix<T> matrix<T>::part (int i1, int i2, int j1, int j2) const
{
    if(i1 > i2  || j1 > j2  || i2-1 > ii || j2-1 > jj){ 
        printf(" Err, matrix<T>:: part,  i1 = %d, i2 = %d, j1 = %d, j2 = %d \n", 
			   i1, i2, j1, j2);    
        err("part (%d, %d), (%d, %d)\n", ii, jj, 0, 0);
    }
	
    matrix<T> d(i2-i1+1, j2-j1+1);    
    for(int i = 0; i < i2-i1+1; i++)
        for(int j = 0; j < j2-j1+1; j++)
			d.aij[i*d.jj + j]  = aij[(i1-1+i)*jj +(j1-1+j)];

    return d;
}

// 行列のある部分に小行列sを代入する (先頭要素は (1,1))
template <class T>
void matrix<T>::pasg (int mi, int mj, const matrix<T> &s)
{
    if(!s.ii ||  !s.jj || ii < s.ii + mi-1  || jj < s.jj + mj-1){        
        printf(" Err, matrix<T>:: pasg,  mi = %d, mj = %d\n", mi, mj);    
        err("pasg, (%d, %d), (%d, %d)\n", ii, jj, s.ii, s.jj);
	}
    
    for(int i=0; i < s.ii; i++)
        for(int j=0; j < s.jj; j++)
            aij[(mi-1+i)*jj + (mj-1+j)] = s.aij[i*s.jj + j];
}

// 行列sのある部分を小行列として得る (先頭要素は (1,1))
template <class T>
matrix<int> matrix<T>::size (void) const
{    
	matrix<int> d(2,1);
	d(1,1) = ii;
	d(2,1) = jj;	   
    return d;    
}

// 行列s1の右に行列s2をつないだ行列を返す (先頭要素は (1,1))
//向后按行连接矩阵
template <class T>
matrix<T> operator | (const matrix<T> &s1, const matrix<T> &s2)
{
    if(!s1.ii || !s1.jj || s1.ii != s2.ii)
        s1.err("operator | (%d, %d), (%d, %d)\n", s1.ii, s1.jj, s2.ii, s2.jj);
    matrix<T> d(s1.ii, s1.jj+s2.jj);
	
	d.pasg(1, 1, s1);	
	d.pasg(1, s1.jj+1, s2);	
	
    return d;
}

// 行列s1の下に行列s2をつないだ行列を返す
//向下按列连接矩阵
template <class T>
matrix<T> operator || (const matrix<T> &s1, const matrix<T> &s2)
{
    if(!s1.ii || !s1.jj || s1.jj != s2.jj)
        s1.err("operator || (%d, %d), (%d, %d)\n", s1.ii, s1.jj, s2.ii, s2.jj);
    matrix<T> d(s1.ii+s2.ii, s1.jj);
	
	d.pasg(1, 1, s1);	
	d.pasg(s1.ii+1, 1, s2);	
	
    return d;
}

// s1とs2との内積を返す (N x 1 行列のみ有効)
template <class T>
T operator & (const matrix<T> &s1, const matrix<T> &s2)
{
    if(s1.ii != s2.ii || !s1.ii || s2.jj != 1 || s1.jj != 1)
        s1.err("operator & (matrix<T>, matrix<T>)(%d, %d), (%d, %d)\n", 
			   s1.ii, s1.jj, s2.ii, s2.jj);

    double x = 0;	
    for(int i = 0; i < s1.ii; i++)
		x += s1.aij[i] * s2.aij[i];
	
    return x;
}

// s1とs2との外積を返す (3 x 1 行列のみ有効)
template <class T>
matrix<T> operator && (const matrix<T> &s1, const matrix<T> &s2)
{
    if(s1.ii != 3 || s2.ii != 3 || s1.jj != 1 || s2.jj != 1)
        s1.err("operator && (matrix<T>, matrix<T>)(%d, %d), (%d, %d)\n", 
			   s1.ii, s1.jj, s2.ii, s2.jj);
	
    matrix<T> d(3, 1);
	
	d.aij[0] = s1.aij[1]*s2.aij[2] - s1.aij[2]*s2.aij[1];
	d.aij[1] = s1.aij[2]*s2.aij[0] - s1.aij[0]*s2.aij[2];
	d.aij[2] = s1.aij[0]*s2.aij[1] - s1.aij[1]*s2.aij[0];

    return d;
}

// s1とs2の各要素の積を要素とする行列を返す (N x 1 行列のみ有効)
template <class T>
matrix<T> operator % (const matrix<T> &s1, const matrix<T> &s2)
{
    if(s1.ii != s2.ii || !s1.ii || s2.jj != 1 || s1.jj != 1)
        s1.err("operator \% (matrix<T>, matrix<T>)(%d, %d), (%d, %d)\n", 
			   s1.ii, s1.jj, s2.ii, s2.jj);
	
    matrix<T> d(s1.ii, 1);
	
    for(int i = 0; i < s1.ii; i++)
		d.aij[i] = s1.aij[i]*s2.aij[i];
	
    return d;
}

// ガウスの消去法により A^{-1}*B を求める
template <class T>
matrix<T> operator > (const matrix<T> &A, const matrix<T> &B)
{
	if(A.ii != B.ii || !A.ii || B.jj != 1)
		A.err("operator > (matrix<T>, matrix<T>)(%d, %d), (%d, %d)\n",
			  A.ii, A.jj, B.ii, B.jj);
	
	matrix<T> a(A), b(B);
	double c, m, n;
	int i, j, k;
	
	for(k = 0; k < A.ii; k++){
		// 部分pivot
		m = 0.0; j = k;
		for(i = k; i < A.ii; i++)
			if(m < (n = fabs(a.aij[i * A.jj + k]))){
				j = i;
				m = n;
			}
		if(j != k){
			a.swapi(k, j);
			b.swapi(k, j);
		}
		
		for(i = k + 1; i < A.ii; i++){
			c = a.aij[i * A.ii + k] / a.aij[k * A.ii + k];
			
			for(j = k; j < A.jj; j++)
				a.aij[i * A.ii + j] -= c * a.aij[k * A.ii + j];
			b.aij[i] -= c * b.aij[k];
		}
	}
	
	for(i = k - 1; i >= 0; i--){
		for(j = i + 1; j < A.jj; j++)
			b.aij[i] -= b.aij[j] * a.aij[i * A.ii + j];
		
		b.aij[i] /=  a.aij[i * A.ii + i];
	}
	
	return b;
}

// 行列の値を配列に代入 (配列の長さに気を付けること)
template <class T>
void operator <<= (T *data, const matrix<T> &s)
{
	if(!s.ii || !s.jj)
        s.err("operator <<= (T *data, const matrix<T> &s), (%d, %d), \n", 
			  s.ii, s.jj, 0, 0);

	size_t size = s.ii * s.jj;
	memcpy(data, s.aij,  size * sizeof(T));
}

// 行列が正則(0)か特異(1)か判定する
template <class T>
int matrix<T>::snglr (T limit) const
{	
    if(!ii || ii != jj)
        err("det (%d, %d)\n", ii, jj, 0, 0);
	
	double result = det();
	if(fabs(result) >= limit)
		return 0;
	else{
#ifdef SNGLR_CHK
		printf("singlular deteriminant : %f\n", result);
#endif
		return 1;
	}
}

// ユークリッドノルムを返す (N x 1 行列のみ有効)
template <class T>
double matrix<T>::norm (void) const  //自作	
{
    if(!ii || jj != 1)
        err("norm (%d, %d)\n", ii, jj, 0, 0);

	double xx = 0.0, x = 0.0;
	for(int i = 0; i < ii; i++)
		xx += (double)(aij[i]*aij[i]);
	x = sqrt(xx);
	
    return x;	
}

// 正規化したベクトルを返す (N x 1 行列のみ有効)
template <class T>
matrix<T> matrix<T>::normal (void) const
{
	if(!ii || jj != 1)
        err("normal (%d, %d)\n", ii, jj, 0, 0);

	matrix<T> s(*this), d(ii, 1);
	double m = s.norm();

	if(m != 0.0)
		for(int i = 0; i < ii; i++)
			d.aij[i] =  s.aij[i] / m;
	
	return d;
}

// キャスト (配列の長さに気を付けること)
template <class T>
matrix<double> matrix<T>::cast_d (void) const  //自作	
{
    if(!ii || !jj)
        err("cast_d (%d, %d)\n", ii, jj, 0, 0);

    matrix<double> d(ii,jj);
    for(int i = 0; i < ii; i++)
		for(int j = 0; j < jj; j++)
			d(i+1,j+1) = (double)aij[i*jj+j];	
    return d;
}

template <class T>
matrix<int> matrix<T>::cast_i (void) const  //自作	
{
    if(!ii || !jj)
        err("operator !() (%d, %d)\n", ii, jj, 0, 0);
	
    matrix<int> d(ii,jj);
    for(int i = 1; i <= ii; i++)
		for(int j = 1; j <= jj; j++)
			d(i+1,j+1) = (int)aij[i*jj+j];	
    return d;
}

typedef matrix<double> MATRIX_D;
typedef matrix<int> MATRIX_I;

extern MATRIX_D Rot(int, double, double);
extern MATRIX_D RotD(int, double, double);
extern MATRIX_D Skew(double, double, double);

extern MATRIX_D MatD31(double, double, double);
extern MATRIX_D MatD33(double, double, double,
					   double, double, double, double, double, double);
extern MATRIX_D MatD61(double, double, double, double, double, double);

extern MATRIX_D Zeros(int, int);
extern MATRIX_D Ones(int, int);
extern MATRIX_D Eye(int);

/* extern int CalcEigenValue(MATRIX_D &, MATRIX_D &, MATRIX_D &); */

#endif
