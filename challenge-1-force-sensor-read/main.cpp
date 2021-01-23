#include <fstream>
#include <iostream>
#include <vector>
#include <bitset>

class matrix{
	// ROW MAJOR!
	int rows;
	int cols;

public:
	double *mat;
	//constructor
	matrix(int m, int n){
		rows = m;
		cols = n;
		mat = new double[m*n];

		for (int i = 0; i < m*n; ++i){
			mat[i] = 0;
		}
	}
	//destructor
	~matrix(){
		delete mat;
	}

	matrix operator + (matrix b){

		matrix m_res(b.rows, b.cols);

		if (rows == b.rows && cols == b.cols){
			for (int x = 0; x < rows*cols; ++x){
				m_res.mat[x] = mat[x] + b.mat[x];
			}
		}
		else{
			for (int x = 0; x < rows*cols; ++x){
				m_res.mat[x] = 0;
			}
			std::cout << "Error! Cannot add matrices with different dimensions!" << std::endl;
		}

		return m_res;
	}
	matrix operator * (const matrix b){

		matrix *m_res = new matrix(rows, b.cols);

		for (int x = 0; x < rows*b.cols; ++x){
			m_res->mat[x] = 0;
		}
		if (cols == b.rows){
			for (int i = 0; i < b.cols; ++i){
				for (int j = 0; j < rows; ++j){
					// i and j select the element in the resulting matrix

					for (int k = 0; k < cols; ++k){
						// k selects the element in the selected row/col
						//								rows A				cols B
						m_res->mat[i + j*b.cols] += mat[j*cols + k]*b.mat[i + k*b.cols];
						//std::cout << "element: " << i + j*b.cols << " a: " << j*cols + k << " b: " << i + k*b.cols << std::endl;
						//std::cout << "element: " << i + j*b.cols << " a: " << mat[j*cols + k] << " b: " << b.mat[i + k*b.cols] << std::endl;
					}
				}
			}
		}
		else{
			std::cout << "Error! Cannot multiply matrices with invalid dimensions!" << std::endl;
		}

		return *m_res;
	}
	matrix operator / (double d){

		for (int i = 0; i < rows*cols; ++i){
			mat[i] = mat[i]/d;
		}

		return *this;
	}
	matrix &operator = (matrix b){

		if (rows == b.rows && cols == b.cols){
			for (int i = 0; i < rows*cols; ++i){
				mat[i] = b.mat[i];
			}
		}
		else{
			std::cout << "Cannot attribute matrices with different dimensions!" << std::endl;
		}

		return *this;
	}
	matrix T(){

		matrix *m_temp = new matrix(rows, cols);
		//double *m_temp = new double[cols*rows];

		for (int i = 0; i < cols; ++i){
			for (int j = 0; j < rows; ++j){
				m_temp->mat[i*rows + j] = mat[i + j*cols];
			}
		}
		m_temp->cols = rows;
		m_temp->rows = cols;

		return *m_temp;
	}

	int get_rows(){
		return rows;
	}
	int get_cols(){
		return cols;
	}

	friend std::ostream& operator<< (std::ostream &os, matrix const &m);
};

std::ostream& operator<< (std::ostream &os, matrix const &m){

	for (int i = 0; i < m.rows; ++i){
		for (int j = 0; j < m.cols; ++j){
			os << m.mat[j + i*m.cols] << " ";
		}
		os << std::endl;
	}

	return os;
};

int main(int argc, const char** argv){

	// open a file in write mode.
	std::fstream source_file;
	source_file.open("C:\\Users\\choiw\\Desktop\\My_Workspace\\cpp_moderno\\torque_sensor\\sensor-log.txt",
			std::fstream::in);

	std::string can1, size, response;
	int opcode;
	int sensor_identifier = 0x7F0;
	int CpF = 0, CpT = 0;	// Counts
	int SGi = 0, SGj = 0;	// Matrix coefficients buffers

	matrix TCM(6, 6);		// Transducer calibration matrix
	int TCM_row;			// TCM matrix row indicator

	matrix SGD(6, 1);		// Strain gauge data
	int status;				// Strain gauge status
	int sgi;				// Strain gauge readings

	matrix loads(6, 1);			// Load matrix

	// check file
	if (source_file.is_open()){
		std::cout << "File successfully opened!" << std::endl;
	}
	else{
		std::cout << "Could not open file!" << std::endl;
		return 1;
	}

//	matrix a(6,6), b(6, 1);
//	for (int x = 0; x < 36; ++x){
//		a.mat[x] = x;
//	}
//	for (int x = 0; x < 6; ++x){
//		b.mat[x] = 1;
//	}
//
//	std::cout << a*b;

	std::cout << "Getting only sensor messages..." << std::endl;
	while (!source_file.eof()){
		// parse lines of the file
		source_file >> can1 >> std::hex >> opcode >> size;
		std::getline(source_file, response);

		// filters the messages from the sensor only
		opcode -= sensor_identifier;
		if ( opcode < 0xF ){
			// sorts the messages
			switch (opcode){
				case 0:		// read SG data (status)
					//std::cout << "Getting status:" << std::endl;

					// Jump a line
					source_file >> can1 >> std::hex >> opcode >> size;
					std::getline(source_file, response);
					opcode -= sensor_identifier;

					status = 0;
					if (opcode == 0){
						// Gets first 2 bytes that contains status
						status = 0;
						for (int i = 0; i < 2; ++i){
							status = status*0x100;
							status += std::stoi(response.substr(2 + i*3, 2), nullptr, 16);
						}
						//std::cout << "opcode: " << opcode << " status: " << std::hex << status << std::endl;
						// Gets last 6 bytes that contains strain gauge data
						for (int i = 0; i < 3; ++i){
							sgi = 0;
							for (int j = 0; j < 2; ++j){
								sgi = sgi*0x100;
								sgi += std::stoi(response.substr(8 + i*6 + j*3, 2), nullptr, 16);
							}
							// std::cout << "opcode: " << opcode << " sgi: " << sgi << std::endl;

							// fills vector
							SGD.mat[i*2] = sgi;
						}
					}

					// Jump a line
					source_file >> can1 >> std::hex >> opcode >> size;
					std::getline(source_file, response);
					opcode -= sensor_identifier;

					if (opcode == 1){
						// Gets 6 bytes that contains strain gauge data
						for (int i = 0; i < 3; ++i){
							sgi = 0;
							for (int j = 0; j < 2; ++j){
								sgi = sgi*0x100;
								sgi += std::stoi(response.substr(2 + i*6 + j*3, 2), nullptr, 16);
							}
							// std::cout << "opcode: " << opcode << " sgi: " << sgi << std::endl;

							// fills vector
							SGD.mat[i*2 + 1] = sgi;
						}
					}

					//std::cout << "rows: " << SGD.get_rows() << "cols: " << SGD.get_cols() << std::endl <<  SGD.T();

					loads = TCM*SGD/CpF;
					std::cout << "Calculated loads: " << loads.T();

					break;
				case 2:		// read (gets matrix and axis)
					std::cout << "Reading matrix:" << std::endl;

					// Reads the entire block of messages containing the matrix (maybe, rs)
					while (opcode == 2){

						// Gets the axis row
						TCM_row = std::stoi(response.substr(2, 2), nullptr, 16);

						// Jump line
						source_file >> can1 >> std::hex >> opcode >> size;
						std::getline(source_file, response);
						opcode -= sensor_identifier;

						// parses a block of 3 messages, corresponding to a complete row (maybe?)
						for (int j = 0; j < 3; ++j){
							SGi = 0;
							SGj = 0;
							// parses the message
							for (int i = 0; i < 4; ++i){
								SGi = SGi*0x100;
								SGi += std::stoi(response.substr(2 + i*3, 2), nullptr, 16);

								SGj = SGj*0x100;
								SGj += std::stoi(response.substr(14 + i*3, 2), nullptr, 16);

							}

							TCM.mat[ TCM_row*TCM.get_cols() + 2 * j] = *(float *)&SGi;
							TCM.mat[ TCM_row*TCM.get_cols() + 2 * j + 1] = *(float *)&SGj;

//							std::cout << "SG0: " << SGi << " SG1: " << SGj << std::endl;
//							std::cout << "SG0: " << std::bitset<32>(SGi) << " SG1: " << std::bitset<32>(SGj) << std::endl;
							std::cout << "Row: " << TCM_row << " SG0: " << *(float *)&SGi << " SG1: " << *(float *)&SGj << std::endl;


							// Jump line
							source_file >> can1 >> std::hex >> opcode >> size;
							std::getline(source_file, response);
							opcode -= sensor_identifier;
						}

					}

					std::cout << TCM.T();

					break;
				case 6:		// Set Active
					break;
				case 7:		// Read (counts per force or torque)
					std::cout << "Counts: ";

					// Jump a line
					source_file >> can1 >> std::hex >> opcode >> size;
					std::getline(source_file, response);
					opcode -= sensor_identifier;

					for (int i = 0; i < 4; ++i){
						CpF = CpF*0x100;
						CpF += std::stoi(response.substr(2 + i*3, 2), nullptr, 16);

						CpT = CpT*0x100;
						CpT += std::stoi(response.substr(14 + i*3, 2), nullptr, 16);
					}
					std::cout << "CpF: " << CpF << " CpT: " << CpT << std::endl;
					break;
				case 8:		// Read Unit
					break;
				case 9:
					break;
			}
			//std::cout << can1 << " " << opcode << " " << size << response << std::endl;
		}
	}

	source_file.close();

	return 0;
}
