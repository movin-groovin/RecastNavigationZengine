#ifndef MATRIX_LIB_H
#define MATRIX_LIB_H

#include "Geometry.h"
#include <initializer_list>
#include <memory>
#include <cassert>
#include <cstring>

namespace matrix
{

template <typename T, template <typename, typename> class Matrix>
class MatrixPool
{
public:
	using MatrixType = Matrix<T, MatrixPool>;
	using DataType = typename MatrixType::DataType;

	struct Tag
	{
		int rowsNum;
		int columnsNum;
	};

	struct Arg
	{
		int rowsNum;
		int columnsNum;
		int matricesNum;

		constexpr Tag tag() const { return {rowsNum, columnsNum}; }
	};

private:
	struct Entry
	{
		int rowsNum;
		int columnsNum;
		int matricesNum;
		int matricesSize;
		std::unique_ptr<DataType* []> m_freeBlocks;
		std::unique_ptr<DataType[]> m_data;
	};

public:
	MatrixPool() = default;
	~MatrixPool()
	{
		m_size = 0;
	}
	MatrixPool(const MatrixPool&) = delete;
	MatrixPool& operator=(const MatrixPool&) = delete;

	bool init(std::initializer_list<Arg> args)
	{
		m_size = (int)args.size();
		m_state.reset(new (std::nothrow) Entry[m_size]);
		if (!m_state)
			return false;

		int i = 0;
		for (const Arg& arg : args) {
			Entry& e = m_state[i++];
			if (e.m_data) {
				return false;
			}

			e.rowsNum = arg.rowsNum;
			e.columnsNum = arg.columnsNum;
			e.matricesNum = 0;
			e.matricesSize = arg.matricesNum;
			e.m_data.reset(new (std::nothrow) DataType[e.rowsNum * e.columnsNum * e.matricesSize]);
			if (!e.m_data) {
				m_state.reset();
				return false;
			}
			e.m_freeBlocks.reset(new (std::nothrow) DataType * [e.matricesSize]);
			if (!e.m_freeBlocks) {
				m_state.reset();
				return false;
			}

			for (int j = 0; j < e.matricesSize; ++j) {
				e.m_freeBlocks[j] = e.m_data.get() + j * e.rowsNum * e.columnsNum;
			}
		}

		return true;
	}

	MatrixType allocMatrix(const Tag maxSize, const Tag realSize)
	{
		if (maxSize.rowsNum < realSize.rowsNum || maxSize.columnsNum < realSize.columnsNum)
			return MatrixType(MatrixType::ERROR_INVALID_POOL_ARGUMENT);

		for (int i = 0; i < m_size; ++i) {
			Entry& e = m_state[i];
			if (e.rowsNum == maxSize.rowsNum && e.columnsNum == maxSize.columnsNum) {
				if (e.matricesNum == e.matricesSize)
					return MatrixType(MatrixType::ERROR_NO_MEMORY);
				return MatrixType(
					this,
					e.m_freeBlocks[e.matricesNum++],
					maxSize.rowsNum,
					maxSize.columnsNum,
					realSize.rowsNum,
					realSize.columnsNum
				);
			}
		}

		return MatrixType(MatrixType::ERROR_NO_SUCH_MATRIX_IN_POOL);
	}

private:
	void freeMatrix(MatrixType* m)
	{
		for (int i = 0; i < m_size; ++i) {
			Entry& e = m_state[i];
			if (e.rowsNum == m->getRowsSize() && e.columnsNum == m->getColumnsSize()) {
				assert(e.matricesNum > 0);
				e.m_freeBlocks[--e.matricesNum] = m->getData();
			}
		}
	}

private:
	int m_size = 0;
	std::unique_ptr<Entry[]> m_state;

	friend MatrixType;
};

template <typename T, typename MatrixPool>
class Matrix
{
public:
	static const int OK = 0;
	static const int ERROR_NO_MEMORY = 1;
	static const int ERROR_NO_SUCH_MATRIX_IN_POOL = 2;
	static const int ERROR_INVALID_POOL_ARGUMENT = 3;
	static const int ERROR_INVERSION = 4;
	static const int ERROR_INVALID_ARGUMENT = 5;

public:
	using DataType = T;
	static_assert(
		std::is_same<DataType, float>::value || std::is_same<DataType, double>::value,
		"Allowed only FP types"
	);
	using MatrixPoolType = MatrixPool;

public:
	Matrix() = default;
	Matrix(const Matrix& ref) {
		m_dat = nullptr;
		m_pool = nullptr;
		if (!ref) {
			m_lastError = ref.m_lastError;
			return;
		}
		Matrix m( ref.m_pool->allocMatrix({ ref.m_rowsSize, ref.m_columnsSize }, {ref.m_rowsNum, ref.m_columnsNum}) );
		if (m) {
			std::memcpy(m.m_dat, ref.m_dat, sizeof(DataType) * m.m_rowsNum * m.m_columnsNum);
		}
		swap(m);
	}
	Matrix& operator=(const Matrix& ref) {
		if (this == &ref)
			return *this;
		free();
		if (!ref) {
			m_lastError = ref.m_lastError;
			return *this;
		}
		Matrix m(ref.m_pool->allocMatrix({ ref.m_rowsSize, ref.m_columnsSize }, { ref.m_rowsNum, ref.m_columnsNum }));
		if (m) {
			std::memcpy(m.m_dat, ref.m_dat, sizeof(DataType) * m.m_rowsNum * m.m_columnsNum);
		}
		swap(m);
		return *this;
	}
	Matrix(Matrix&& ref) {
		m_dat = nullptr;
		m_pool = nullptr;
		swap(ref);
	}
	Matrix& operator=(Matrix&& ref) {
		if (this == &ref)
			return *this;
		free();
		swap(ref);
		return *this;
	}
	~Matrix()
	{
		free();
		//MatrixPool::Tag a;
	}
	void swap(Matrix& ref)
	{
		std::swap(m_rowsSize, ref.m_rowsSize);
		std::swap(m_columnsSize, ref.m_columnsSize);
		std::swap(m_rowsNum, ref.m_rowsNum);
		std::swap(m_columnsNum, ref.m_columnsNum);
		std::swap(m_dat, ref.m_dat);
		std::swap(m_pool, ref.m_pool);
		std::swap(m_lastError, ref.m_lastError);
	}

	int getRowsNum() const { return m_rowsNum; }
	int getColumnsNum() const { return m_columnsNum; }
	int getRowsSize() const { return m_rowsSize; }
	int getColumnsSize() const { return m_columnsSize; }
	DataType* getData() { return m_dat; }
	const DataType* getData() const { return m_dat; }
	int getLastError() const { return m_lastError; }
	explicit operator bool() const { return m_pool && m_lastError == OK; }

	bool setData(const DataType* data, const int dataNum)
	{
		if (m_rowsNum * m_columnsNum != dataNum) {
			return false;
		}
		if (!static_cast<const Matrix&>(*this)) {
			return false;
		}
		std::memcpy(m_dat, data, sizeof(DataType) * dataNum);
		return true;
	}

	Matrix operator*(const Matrix& ref)
	{
		int lastError = OK;
		if (!checkState(ref, lastError)) {
			return Matrix(lastError);
		}
		if (m_columnsNum != ref.m_rowsNum) {
			return Matrix(ERROR_INVALID_ARGUMENT);
		}
		Matrix m = m_pool->allocMatrix({ m_rowsSize, ref.m_columnsSize }, { m_rowsNum, ref.m_columnsNum });
		if (!m) {
			return m;
		}

		for (int i = 0; i < m_rowsNum; ++i) {
			for (int j = 0; j < ref.m_columnsNum; ++j) {
				DataType tmp = DataType();
				for (int k = 0; k < m_columnsNum; ++k)
				{
					tmp += m_dat[i * m_columnsNum + k] * ref.m_dat[k * ref.m_columnsNum + j];
				}
				m.m_dat[i * ref.m_columnsNum + j] = tmp;
			}
		}

		return m;
	}

	Matrix transpose()
	{
		if (!static_cast<const Matrix&>(*this)) {
			return Matrix(getLastError());
		}
		Matrix m = m_pool->allocMatrix({ m_columnsSize, m_rowsSize }, { m_columnsNum, m_rowsNum });
		if (!m) {
			return m;
		}

		for (int i = 0; i < m_rowsNum; ++i)
		{
			for (int j = 0; j < m_columnsNum; ++j)
			{
				m.m_dat[j * m_rowsNum + i] = m_dat[i * m_columnsNum + j];
			}
		}

		return m;
	}

	Matrix inverse3x3()
	{
		if (!static_cast<const Matrix&>(*this)) {
			return Matrix(getLastError());
		}
		if (m_rowsNum != m_columnsNum || m_rowsNum != 3)
		{
			return Matrix(ERROR_INVALID_ARGUMENT);
		}

		DataType det = m_dat[0] * m_dat[4] * m_dat[8] + m_dat[1] * m_dat[5] * m_dat[6] + m_dat[3] * m_dat[7] * m_dat[2]
			- m_dat[2] * m_dat[4] * m_dat[6] - m_dat[3] * m_dat[1] * m_dat[8] - m_dat[7] * m_dat[5] * m_dat[0];
		if (det != det || std::isinf(det) || std::fabs(det) < geometry::Constants::EPS)
		{
			return Matrix(ERROR_INVERSION);
		}
		Matrix m = m_pool->allocMatrix({ m_rowsSize, m_columnsSize }, { m_rowsNum, m_columnsNum });
		if (!m) {
			return m;
		}

		//const DataType invDet = DataType(1) / det;
		m.m_dat[0] = (m_dat[4] * m_dat[8] - m_dat[5] * m_dat[7]) / det;
		m.m_dat[1] = -(m_dat[3] * m_dat[8] - m_dat[5] * m_dat[6]) / det;
		m.m_dat[2] = (m_dat[3] * m_dat[7] - m_dat[4] * m_dat[6]) / det;
		m.m_dat[3] = -(m_dat[1] * m_dat[8] - m_dat[2] * m_dat[7]) / det;
		m.m_dat[4] = (m_dat[0] * m_dat[8] - m_dat[2] * m_dat[6]) / det;
		m.m_dat[5] = -(m_dat[0] * m_dat[7] - m_dat[1] * m_dat[6]) / det;
		m.m_dat[6] = (m_dat[1] * m_dat[5] - m_dat[2] * m_dat[4]) / det;
		m.m_dat[7] = -(m_dat[0] * m_dat[5] - m_dat[2] * m_dat[3]) / det;
		m.m_dat[8] = (m_dat[0] * m_dat[4] - m_dat[1] * m_dat[3]) / det;

		return m;
	}

	Matrix operator+(const Matrix& ref)
	{
		int lastError = OK;
		if (!checkState(ref, lastError)) {
			return Matrix(lastError);
		}
		if (m_rowsNum != ref.m_rowsNum || m_columnsNum != ref.m_columnsNum)
		{
			return Matrix(ERROR_INVALID_ARGUMENT);
		}
		Matrix m = *this;
		if (!m) {
			return m;
		}

		m.binaryOpApplay([](const DataType v1, const DataType v2) { return v1 + v2 }, ref);

		return m;
	}

	Matrix operator-(const Matrix& ref)
	{
		int lastError = OK;
		if (!checkState(ref, lastError)) {
			return Matrix(lastError);
		}
		if (m_rowsNum != ref.m_rowsNum || m_columnsNum != ref.m_columnsNum)
		{
			return Matrix(ERROR_INVALID_ARGUMENT);
		}
		Matrix m = *this;
		if (!m) {
			return m;
		}

		m.binaryOpApplay([](const DataType v1, const DataType v2) { return v1 - v2; }, ref);

		return m;
	}

	DataType sumSquared() const
	{
		if (!static_cast<const Matrix&>(*this)) {
			return DataType(FLT_MAX);
		}
		return std::accumulate(
			m_dat,
			m_dat + m_rowsNum * m_columnsNum,
			DataType(),
			[](const DataType v1, const DataType v2) { return v1 + v2 * v2; }
		);
	}

private:
	explicit Matrix(
		MatrixPoolType* pool,
		DataType* dat,
		const int m_rowsSize,
		const int m_columnsSize,
		const int rowsNum,
		const int columnsNum
	):
		m_rowsSize(m_rowsSize),
		m_columnsSize(m_columnsSize),
		m_rowsNum(rowsNum),
		m_columnsNum(columnsNum),
		m_dat(dat),
		m_pool(pool),
		m_lastError(OK)
	{}

	explicit Matrix(const int lastError) { m_lastError = lastError; }

	void free()
	{
		if (m_pool)
			m_pool->freeMatrix(this);
		m_pool = nullptr;
		m_dat = nullptr;
	}

	template <typename F>
	void binaryOpApplay(const F& f, const Matrix& ref)
	{
		for (int i = 0; i < m_rowsNum; ++i)
		{
			for (int j = 0; j < m_columnsNum; ++j)
			{
				m_dat[i * m_columnsNum + j] = f(m_dat[i * m_columnsNum + j], ref.m_dat[i * m_columnsNum + j]);
			}
		}
	}

	bool checkState(const Matrix& ref, int& lastError)
	{
		if (!static_cast<const Matrix&>(*this))
		{
			lastError = getLastError();
			return false;
		}
		if (!ref)
		{
			lastError = ref.getLastError();
			return false;
		}
		return true;
	}

private:
	int m_rowsSize = 0;
	int m_columnsSize = 0;
	int m_rowsNum = 0;
	int m_columnsNum = 0;
	DataType* m_dat = nullptr;
	MatrixPoolType* m_pool = nullptr;
	int m_lastError = OK;

	friend MatrixPoolType;
};

using MatrixPoolDouble = MatrixPool<double, Matrix>;
using MatrixDouble = MatrixPoolDouble::MatrixType;

} // namespace matrix

#endif // MATRIX_LIB_H