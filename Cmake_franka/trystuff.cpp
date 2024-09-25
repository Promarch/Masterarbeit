#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

// Thread-safe queue to store vectors
std::queue<std::vector<std::array<int, 3>>> dataQueue;
std::mutex mtx;
std::condition_variable cv;
bool stopWriting = false;

template <typename T, std::size_t N>
void writeVectorsToFile() {
    std::ofstream outFile("output.txt");
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file!" << std::endl;
        return;
    }

    while (true) {
        std::vector<std::array<T, N>> data;
        
        // Lock and wait for data or stop signal
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [] { return !dataQueue.empty() || stopWriting; });
            if (stopWriting && dataQueue.empty()) {
                break;
            }
            data = dataQueue.front();
            dataQueue.pop();
        }

        // Write the data to the file (outside of the critical section)
        for (const auto& arr : data) {
            for (const auto& element : arr) {
                outFile << element << " ";
            }
            outFile << std::endl;
        }
    }
    
    outFile.close();
}

int main() {
    // Start the file writing thread
    std::thread writerThread(writeVectorsToFile<int, 3>);
    
    // Simulate a control loop that runs every 1ms
    for (int i = 0; i < 1000; ++i) {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Simulate control loop operation
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        
        // Generate some data
        std::vector<std::array<int, 3>> vec = {
            {i, i+1, i+2},
            {i+3, i+4, i+5}
        };

        // Push data to the queue in a thread-safe manner
        {
            std::lock_guard<std::mutex> lock(mtx);
            dataQueue.push(vec);
        }
        
        // Notify the writer thread that new data is available
        cv.notify_one();
        
        // Control loop time management to stay under 1ms
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        if (elapsed.count() < 0.001) {
            std::this_thread::sleep_for(std::chrono::duration<double>(0.001 - elapsed.count()));
        }
    }

    // Signal to stop writing and join the thread
    {
        std::lock_guard<std::mutex> lock(mtx);
        stopWriting = true;
    }
    cv.notify_one();
    writerThread.join();

    return 0;
}
