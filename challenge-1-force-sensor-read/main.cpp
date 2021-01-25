#include <fstream>
#include <iostream>
#include <vector>
#include <bitset>

class matrix : public std::vector<double>{
	// ROW MAJOR!
	int rows;
	int cols;

public:
	// overloaded constructor
	matrix(int m, int n){
		rows = m;
		cols = n;

		// Resize the vector to allocate the matrix
		resize(m*n);
	}

	matrix operator + (const matrix b){

		matrix m_res(rows, cols);

		if (rows == b.rows && cols == b.cols){
			for (int x = 0; x < rows*cols; ++x){
				m_res.at(x) = at(x) + b.at(x);
			}
		}
		else{
			for (int x = 0; x < rows*cols; ++x){
				m_res.at(x) = 0;
			}
			std::cout << "Error! Cannot add matrices with different dimensions!" << "\n";
		}

		return m_res;
	}
	matrix operator * (const matrix b){

		matrix m_res = matrix(rows, b.cols);

		for (int x = 0; x < rows*b.cols; ++x){
			m_res.at(x) = 0;
		}
		if (cols == b.rows){
			for (int i = 0; i < b.cols; ++i){
				for (int j = 0; j < rows; ++j){
					// i and j select the element in the resulting matrix

					for (int k = 0; k < cols; ++k){
						// k selects the element in the selected row/col
						//								rows A				cols B
						m_res.at(i + j*b.cols) += at(j*cols + k) * b.at(i + k*b.cols);
						//std::cout << "element: " << i + j*b.cols << " a: " << j*cols + k << " b: " << i + k*b.cols << "\n";
						//std::cout << "element: " << i + j*b.cols << " a: " << mat[j*cols + k] << " b: " << b.mat[i + k*b.cols] << "\n";
					}
				}
			}
		}
		else{
			std::cout << "Error! Cannot multiply matrices with invalid dimensions!" << "\n";
		}

		return m_res;
	}
	matrix operator / (double d){

		for (int i = 0; i < rows*cols; ++i){
			at(i) = at(i)/d;
		}

		return *this;
	}

	matrix T(){

		matrix m_temp = matrix(rows, cols);
		//double *m_temp = new double[cols*rows];

		for (int i = 0; i < cols; ++i){
			for (int j = 0; j < rows; ++j){
				m_temp.at(i*rows + j) = at(i + j*cols);
			}
		}
		m_temp.cols = rows;
		m_temp.rows = cols;

		return m_temp;
	}

	int get_rows(){
		return rows;
	}
	int get_cols(){
		return cols;
	}

	double i_access(int m, int n){
		// access matrix elements through common indices: row, col
		return at( (n - 1) + (m - 1) *cols );
	}
	void i_change(int m, int n, double x){
		// Change matrix elements through common indices: row, col
		at( (n - 1) + (m - 1) *cols ) = x;
	}

	friend std::ostream& operator<< (std::ostream &os, matrix const &m){
		for (int i = 0; i < m.rows; ++i){
			for (int j = 0; j < m.cols; ++j){
				os << m.at(j + i*m.cols) << " ";
			}
			os << "\n";
		}

		return os;
	};

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
		std::cout << "File successfully opened!" << "\n";
	}
	else{
		std::cout << "Could not open file!" << "\n";
		return 1;
	}

	std::cout << "Getting only sensor messages..." << "\n";
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
					//std::cout << "Getting status:" << "\n";

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
						//std::cout << "opcode: " << opcode << " status: " << std::hex << status << "\n";
						// Gets last 6 bytes that contains strain gauge data
						for (int i = 0; i < 3; ++i){
							sgi = 0;
							for (int j = 0; j < 2; ++j){
								sgi = sgi*0x100;
								sgi += std::stoi(response.substr(8 + i*6 + j*3, 2), nullptr, 16);
							}
							// std::cout << "opcode: " << opcode << " sgi: " << sgi << "\n";

							// fills vector
							SGD.at(i*2) = sgi;
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
							// std::cout << "opcode: " << opcode << " sgi: " << sgi << "\n";

							// fills vector
							SGD.at(i*2 + 1) = sgi;
						}
					}

					//std::cout << "rows: " << SGD.get_rows() << "cols: " << SGD.get_cols() << "\n" <<  SGD.T();

					loads = TCM*SGD/CpF;
					std::cout << "Calculated loads: " << loads.T();

					break;
				case 2:		// read (gets matrix and axis)
					std::cout << "Reading matrix:" << "\n";

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

							TCM.at( TCM_row*TCM.get_cols() + 2 * j) = *(float *)&SGi;
							TCM.at( TCM_row*TCM.get_cols() + 2 * j + 1) = *(float *)&SGj;

//							std::cout << "SG0: " << SGi << " SG1: " << SGj << "\n";
//							std::cout << "SG0: " << std::bitset<32>(SGi) << " SG1: " << std::bitset<32>(SGj) << "\n";
							std::cout << "Row: " << TCM_row << " SG0: " << *(float *)&SGi << " SG1: " << *(float *)&SGj << "\n";


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
					std::cout << "CpF: " << CpF << " CpT: " << CpT << "\n";
					break;
				case 8:		// Read Unit
					break;
				case 9:
					break;
			}
		}
	}

	source_file.close();

	return 0;
}
