#include <string>
#include <fstream>
#include <memory>
#include "msgs/log_message.h"

/**
 * @brief Class for writing data to a CSV file.
 */
class CsvWriter {
    std::string   m_header; /**< Header string for the CSV file. */
    std::string   m_fileName; /**< Name of the CSV file. */
    std::ofstream m_file; /**< Output file stream. */

public:
    /**
     * @brief Constructor.
     * @param header The header string for the CSV file.
     */
    CsvWriter(const std::string& header){
        m_header = header + "\n";
    }

    /**
     * @brief Destructor. Closes the file.
     */
    ~CsvWriter(){
        close();
    }

    /**
     * @brief Opens a CSV file for writing.
     * @param filename The name of the file to be opened.
     * @return true if the file was opened successfully, false otherwise.
     */
    bool open(const std::string& filename ) {
        m_fileName = filename;
        m_file.open(m_fileName);
        
        m_file << m_header;
        return true;
    }

    /**
     * @brief Appends data to an existing CSV file or creates a new one if it doesn't exist.
     * @param filename The name of the file to which data will be appended.
     * @return true if data was successfully appended, false otherwise.
     */
    bool append(const std::string& filename ) {
        m_fileName = filename;
        std::ifstream infile(m_fileName);
        bool newfile = !infile.good();

        m_file.open(m_fileName, std::ios_base::app);
        if (newfile){
            m_file << m_header;
        }
        return true;
    }

    /**
     * @brief Flushes the contents of the file stream.
     */
    void flush(){
        m_file.flush();
    }

    /**
     * @brief Closes the CSV file.
     */
    void close() {
        m_file.close();        
    }

    /**
     * @brief Limits the number of lines in the CSV file.
     * @param maxLines The maximum number of lines to keep in the file.
     */
    void limitLines(int maxLines){
        // check the number of lines in file (seems to be quicker than loop on large file)
        std::string command = "wc -l < " + m_fileName + " > linecount.txt";
        std::system(command.c_str());

        // Read the line count from the temp file
        std::ifstream countFile("linecount.txt");
        int lineCount;
        countFile >> lineCount;
        countFile.close();
        std::remove("linecount.txt");

        if (lineCount > maxLines) {
            int lineToRemove = lineCount - maxLines; // remove excedings lines

            m_file.close();

            std::ifstream inFile(m_fileName);
            std::ofstream outFile(m_fileName + ".temp");

            std::string line;
            int countLinesToRemove = 0;

            std::string lineHeader = "stamp,level,name,msg,file,function,line\n";
            outFile << lineHeader;

            while (std::getline(inFile, line)) {
                countLinesToRemove++;
                if (countLinesToRemove <= lineToRemove) {
                    // skip lines
                    continue;
                }
                outFile << line << "\n";
            }

            inFile.close();
            outFile.close();

            // Replace the original file with the new file
            std::remove(m_fileName.c_str());
            std::rename(std::string(m_fileName + ".temp").c_str(), m_fileName.c_str());

            // open in append mode
            m_file.open(m_fileName, std::ios_base::app);
        }
    }
    
    /**
     * @brief Writes a log message to the CSV file.
     * @param msg A shared pointer to the log message.
     */
    void write(std::shared_ptr<LogMessage> msg){
        const char delimiter  = 0x60; // character used as delimiter= '`'

        std::string line  = std::to_string(msg->stamp) + delimiter +
                            std::to_string(msg->level) + delimiter +
                            msg->name                  + delimiter +
                            msg->msg                   + delimiter +
                            msg->file                  + delimiter +
                            msg->function              + delimiter +
                            msg->line                  + '\n';
        
        m_file << line;
    }
};