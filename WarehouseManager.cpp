#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>
#include <unordered_map>
#include <cctype>
using namespace std;

const double INF = 1e18;
const int NUM_PRIORITIES = 5;

struct Warehouse {
    string id;
    string city;
};

struct Edge {
    double distance;
    double time;
    double cost;
};

struct Delivery {
    string id;
    string origin;
    string dest;
    int priority_value;
    string route_priority;
};

struct PathNode {
    string warehouse_id;
    PathNode* next;
    PathNode(string id) : warehouse_id(id), next(nullptr) {}
};

struct RouteInfo {
    double total_distance;
    double total_time;
    double total_cost;
    PathNode* path_head;
};

struct HeapElement {
    Delivery delivery;
    RouteInfo* route;
    HeapElement(const Delivery& d, RouteInfo* r) : delivery(d), route(r) {}
};

struct AVLNode {
    Delivery delivery;
    AVLNode* left;
    AVLNode* right;
    int height;
    AVLNode(const Delivery& d) : delivery(d), left(nullptr), right(nullptr), height(1) {}
};

// Custom Stack implementation for DFS
template <typename T>
class CustomStack {
private:
    struct Node {
        T data;
        Node* next;
        Node(T val) : data(val), next(nullptr) {}
    };
    Node* top;

public:
    CustomStack() : top(nullptr) {}
    
    void push(T val) {
        Node* newNode = new Node(val);
        newNode->next = top;
        top = newNode;
    }
    
    void pop() {
        if (isEmpty()) return;
        Node* temp = top;
        top = top->next;
        delete temp;
    }
    
    T peek() {
        if (isEmpty()) throw runtime_error("Stack is empty");
        return top->data;
    }
    
    bool isEmpty() {
        return top == nullptr;
    }
    
    ~CustomStack() {
        while (!isEmpty()) pop();
    }
};

// Custom Queue implementation for BFS
template <typename T>
class CustomQueue {
private:
    struct Node {
        T data;
        Node* next;
        Node(T val) : data(val), next(nullptr) {}
    };
    Node *front, *rear;

public:
    CustomQueue() : front(nullptr), rear(nullptr) {}
    
    void enqueue(T val) {
        Node* newNode = new Node(val);
        if (isEmpty()) {
            front = rear = newNode;
        } else {
            rear->next = newNode;
            rear = newNode;
        }
    }
    
    void dequeue() {
        if (isEmpty()) return;
        Node* temp = front;
        front = front->next;
        if (front == nullptr) rear = nullptr;
        delete temp;
    }
    
    T peek() {
        if (isEmpty()) throw runtime_error("Queue is empty");
        return front->data;
    }
    
    bool isEmpty() {
        return front == nullptr;
    }
    
    ~CustomQueue() {
        while (!isEmpty()) dequeue();
    }
};

class AVLTree {
private:
    AVLNode* root;

    int height(AVLNode* node) {
        return node ? node->height : 0;
    }

    int balanceFactor(AVLNode* node) {
        return node ? height(node->left) - height(node->right) : 0;
    }

    void updateHeight(AVLNode* node) {
        if (node) node->height = 1 + max(height(node->left), height(node->right));
    }

    AVLNode* rotateRight(AVLNode* y) {
        AVLNode* x = y->left;
        AVLNode* T2 = x->right;
        x->right = y;
        y->left = T2;
        updateHeight(y);
        updateHeight(x);
        return x;
    }

    AVLNode* rotateLeft(AVLNode* x) {
        AVLNode* y = x->right;
        AVLNode* T2 = y->left;
        y->left = x;
        x->right = T2;
        updateHeight(x);
        updateHeight(y);
        return y;
    }

    AVLNode* balance(AVLNode* node) {
        if (!node) return node;
        updateHeight(node);
        int bf = balanceFactor(node);
        if (bf > 1) {
            if (balanceFactor(node->left) < 0)
                node->left = rotateLeft(node->left);
            return rotateRight(node);
        }
        if (bf < -1) {
            if (balanceFactor(node->right) > 0)
                node->right = rotateRight(node->right);
            return rotateLeft(node);
        }
        return node;
    }

    AVLNode* insert(AVLNode* node, const Delivery& delivery) {
        if (!node) return new AVLNode(delivery);
        if (delivery.id < node->delivery.id)
            node->left = insert(node->left, delivery);
        else if (delivery.id > node->delivery.id)
            node->right = insert(node->right, delivery);
        else 
            return node;
        return balance(node);
    }

    AVLNode* findMin(AVLNode* node) {
        while (node && node->left) node = node->left;
        return node;
    }

    AVLNode* remove(AVLNode* node, string id) {
        if (!node) return nullptr;
        if (id < node->delivery.id)
            node->left = remove(node->left, id);
        else if (id > node->delivery.id)
            node->right = remove(node->right, id);
        else {
            if (!node->left || !node->right) {
                AVLNode* temp = node->left ? node->left : node->right;
                if (!temp) {
                    temp = node;
                    node = nullptr;
                } else {
                    *node = *temp;
                }
                delete temp;
            } else {
                AVLNode* temp = findMin(node->right);
                node->delivery = temp->delivery;
                node->right = remove(node->right, temp->delivery.id);
            }
        }
        return balance(node);
    }

    Delivery* search(AVLNode* node, string id) {
        if (!node) return nullptr;
        if (id < node->delivery.id) return search(node->left, id);
        if (id > node->delivery.id) return search(node->right, id);
        return &node->delivery;
    }

    void clear(AVLNode* node) {
        if (node) {
            clear(node->left);
            clear(node->right);
            delete node;
        }
    }

public:
    AVLTree() : root(nullptr) {}
    ~AVLTree() { clear(root); }

    void insert(const Delivery& delivery) {
        root = insert(root, delivery);
    }

    void remove(string id) {
        root = remove(root, id);
    }

    Delivery* search(string id) {
        return search(root, id);
    }
};

class MaxHeap {
private:
    vector<HeapElement> heap;

    void heapifyUp(int index) {
        while (index > 0) {
            int parent = (index - 1) / 2;
            if (heap[parent].delivery.priority_value < heap[index].delivery.priority_value) {
                swap(heap[parent], heap[index]);
                index = parent;
            } else {
                break;
            }
        }
    }

    void heapifyDown(int index) {
        int size = heap.size();
        while (true) {
            int left = 2 * index + 1;
            int right = 2 * index + 2;
            int largest = index;

            if (left < size && heap[left].delivery.priority_value > heap[largest].delivery.priority_value)
                largest = left;
            if (right < size && heap[right].delivery.priority_value > heap[largest].delivery.priority_value)
                largest = right;

            if (largest != index) {
                swap(heap[index], heap[largest]);
                index = largest;
            } else {
                break;
            }
        }
    }

public:
    bool empty() const {
        return heap.empty();
    }

    void push(HeapElement element) {
        heap.push_back(element);
        heapifyUp(heap.size() - 1);
    }

    HeapElement pop() {
        HeapElement top = heap.front();
        heap[0] = heap.back();
        heap.pop_back();
        if (!heap.empty()) heapifyDown(0);
        return top;
    }
};

class WarehouseManager {
private:
    vector<Warehouse> warehouses;
    vector<vector<Edge>> adjMatrix;
    vector<AVLTree> deliveryTrees;
    MaxHeap deliveryHeap;
    vector<string> warehouseIndexMap;
    unordered_map<string, int> warehouseIDToIndex;
    unordered_map<string, int> deliveryToPriorityMap;

    int getWarehouseIndex(const string& id) {
        auto it = warehouseIDToIndex.find(id);
        if (it != warehouseIDToIndex.end()) 
            return it->second;
        return -1;
    }

    int addWarehouse(const string& id, const string& city) {
        int index = getWarehouseIndex(id);
        if (index == -1) {
            warehouses.push_back({id, city});
            warehouseIndexMap.push_back(id);
            warehouseIDToIndex[id] = warehouses.size() - 1;
            index = warehouses.size() - 1;
            for (auto& row : adjMatrix) 
                row.resize(warehouses.size(), {INF, INF, INF});
            adjMatrix.resize(warehouses.size(), vector<Edge>(warehouses.size(), {INF, INF, INF}));
        }
        return index;
    }

    void addConnection(const string& from, const string& to, double dist, double time, double cost) {
        int fromIdx = getWarehouseIndex(from);
        int toIdx = getWarehouseIndex(to);
        if (fromIdx == -1 || toIdx == -1) return;
        adjMatrix[fromIdx][toIdx] = {dist, time, cost};
        adjMatrix[toIdx][fromIdx] = {dist, time, cost};
    }

    RouteInfo* dijkstra(int originIdx, int destIdx, const string& route_priority) {
        int n = warehouses.size();
        vector<double> dist(n, INF);
        vector<int> prev(n, -1);
        vector<bool> visited(n, false);
        dist[originIdx] = 0;

        auto cmp = [](const pair<double, int>& a, const pair<double, int>& b) {
            return a.first > b.first;
        };
        priority_queue<pair<double, int>, vector<pair<double, int>>, decltype(cmp)> pq(cmp);
        pq.push({0, originIdx});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();
            if (visited[u]) continue;
            visited[u] = true;

            for (int v = 0; v < n; ++v) {
                if (adjMatrix[u][v].cost == INF) continue;
                double weight;
                if (route_priority == "distance") {
                    weight = adjMatrix[u][v].distance;
                } else if (route_priority == "time") {
                    weight = adjMatrix[u][v].time;
                } else {
                    weight = adjMatrix[u][v].cost;
                }
                if (weight == INF) continue;
                double newDist = dist[u] + weight;
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    prev[v] = u;
                    pq.push({newDist, v});
                }
            }
        }

        RouteInfo* route = new RouteInfo();
        route->total_distance = 0;
        route->total_time = 0;
        route->total_cost = 0;
        if (dist[destIdx] == INF) return route;

        vector<int> path_indices;
        for (int at = destIdx; at != -1; at = prev[at]) {
            path_indices.push_back(at);
        }
        reverse(path_indices.begin(), path_indices.end());

        PathNode* head = nullptr;
        PathNode* tail = nullptr;
        for (int i = 0; i < path_indices.size(); i++) {
            int idx = path_indices[i];
            PathNode* node = new PathNode(warehouseIndexMap[idx]);
            if (head == nullptr) {
                head = node;
            } else {
                tail->next = node;
            }
            tail = node;
            if (i > 0) {
                int prevIdx = path_indices[i-1];
                route->total_distance += adjMatrix[prevIdx][idx].distance;
                route->total_time += adjMatrix[prevIdx][idx].time;
                route->total_cost += adjMatrix[prevIdx][idx].cost;
            }
        }
        route->path_head = head;
        return route;
    }

    void primMST(int startIdx) {
        int n = warehouses.size();
        if (n == 0 || startIdx < 0 || startIdx >= n) return;
        vector<double> key(n, INF);
        vector<int> parent(n, -1);
        vector<bool> inMST(n, false);
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
        key[startIdx] = 0;
        pq.push({0, startIdx});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();
            inMST[u] = true;
            for (int v = 0; v < n; ++v) {
                double weight = adjMatrix[u][v].cost;
                if (weight != INF && !inMST[v] && weight < key[v]) {
                    key[v] = weight;
                    parent[v] = u;
                    pq.push({key[v], v});
                }
            }
        }

        cout << "Minimum Spanning Tree (MST) Edges:\n";
        double total_cost = 0;
        for (int i = 0; i < n; ++i) {
            if (i == startIdx) continue;
            if (parent[i] != -1) {
                cout << warehouseIndexMap[parent[i]] << " - " << warehouseIndexMap[i] << " : Cost = " << adjMatrix[parent[i]][i].cost << "\n";
                total_cost += adjMatrix[parent[i]][i].cost;
            }
        }
        cout << "Total MST Cost: " << total_cost << "\n\n";
    }

public:
    WarehouseManager() {
        deliveryTrees.resize(NUM_PRIORITIES);
    }

    void loadWarehouses(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Error opening file: " << filename << endl;
            return;
        }
        string line;
        getline(file, line);
        while (getline(file, line)) {
            if (line.empty()) continue;
            stringstream ss(line);
            string id, city;
            if (!getline(ss, id, ',') || !getline(ss, city)) {
                cerr << "Error reading line: " << line << endl;
                continue;
            }
            if (!city.empty() && city.back() == '\r') {
                city.pop_back();
            }
            addWarehouse(id, city);
        }
    }

    void loadConnections(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Error opening file: " << filename << endl;
            return;
        }
        string line;
        getline(file, line);
        while (getline(file, line)) {
            if (line.empty()) continue;
            stringstream ss(line);
            string from, to, dist_str, time_str, cost_str;
            if (!getline(ss, from, ',') || !getline(ss, to, ',') || !getline(ss, dist_str, ',') || !getline(ss, time_str, ',') || !getline(ss, cost_str)) {
                cerr << "Error reading line: " << line << endl;
                continue;
            }
            if (!cost_str.empty() && cost_str.back() == '\r') {
                cost_str.pop_back();
            }
            try {
                double dist = stod(dist_str);
                double time = stod(time_str);
                double cost = stod(cost_str);
                addConnection(from, to, dist, time, cost);
            } catch (const exception& e) {
                cerr << "Error processing connection: " << from << "->" << to << " - " << e.what() << endl;
            }
        }
    }

    void loadDeliveries(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Error opening file: " << filename << endl;
            return;
        }
        string line;
        getline(file, line);
        while (getline(file, line)) {
            if (line.empty()) continue;
            stringstream ss(line);
            string id, origin, dest, priority_str, route_priority;
            if (!getline(ss, id, ',') || !getline(ss, origin, ',') || !getline(ss, dest, ',') || !getline(ss, priority_str, ',') || !getline(ss, route_priority)) {
                cerr << "Error reading line: " << line << endl;
                continue;
            }
            if (!route_priority.empty() && route_priority.back() == '\r') {
                route_priority.pop_back();
            }
            try {
                int priority_val = stoi(priority_str);
                if (priority_val < 1 || priority_val > NUM_PRIORITIES) {
                    cerr << "Invalid priority value: " << priority_val << " for delivery " << id << endl;
                    continue;
                }
                Delivery d{id, origin, dest, priority_val, route_priority};
                deliveryTrees[priority_val - 1].insert(d);
                deliveryToPriorityMap[id] = priority_val;
            } catch (const exception& e) {
                cerr << "Error processing delivery: " << id << " - " << e.what() << endl;
            }
        }
    }

    void scheduleDelivery(const string& deliveryId) {
        auto it = deliveryToPriorityMap.find(deliveryId);
        if (it == deliveryToPriorityMap.end()) {
            cerr << "Delivery not found: " << deliveryId << endl;
            return;
        }
        int priority_val = it->second;
        Delivery* delivery = deliveryTrees[priority_val - 1].search(deliveryId);
        if (!delivery) {
            cerr << "Delivery not found in AVL tree: " << deliveryId << endl;
            return;
        }
        int originIdx = getWarehouseIndex(delivery->origin);
        int destIdx = getWarehouseIndex(delivery->dest);
        if (originIdx == -1 || destIdx == -1) {
            cerr << "Invalid warehouse for delivery: " << deliveryId << endl;
            return;
        }
        RouteInfo* route = dijkstra(originIdx, destIdx, delivery->route_priority);
        deliveryHeap.push(HeapElement(*delivery, route));
        deliveryTrees[priority_val - 1].remove(deliveryId);
        deliveryToPriorityMap.erase(deliveryId);
    }

    void processDeliveries() {
        if (deliveryHeap.empty()) {
            cout << "No deliveries to process!\n";
            return;
        }
        cout << "\n===== PROCESSING DELIVERIES =====\n";
        while (!deliveryHeap.empty()) {
            HeapElement elem = deliveryHeap.pop();
            Delivery d = elem.delivery;
            RouteInfo* r = elem.route;
            cout << "Processing delivery: " << d.id << " (Priority: " << d.priority_value << ", Route Priority: " << d.route_priority << ")\n";
            cout << "Route: ";
            PathNode* node = r->path_head;
            if (node == nullptr) {
                cout << "No path found!";
            } else {
                while (node) {
                    cout << node->warehouse_id;
                    node = node->next;
                    if (node) {
                        cout << " -> ";
                    }
                }
            }
            cout << "\nCost: " << r->total_cost << " | Time: " << r->total_time << " | Distance: " << r->total_distance << endl << endl;
            node = r->path_head;
            while (node) {
                PathNode* temp = node;
                node = node->next;
                delete temp;
            }
            delete r;
        }
        cout << "All deliveries processed!\n";
    }

    void computeMST(const string& startWarehouse) {
        int startIdx = getWarehouseIndex(startWarehouse);
        if (startIdx == -1) {
            cerr << "Warehouse not found: " << startWarehouse << endl;
            return;
        }
        primMST(startIdx);
    }
    
    void BFS(const string& startWarehouse) {
        int startIdx = getWarehouseIndex(startWarehouse);
        if (startIdx == -1) {
            cout << "Warehouse not found: " << startWarehouse << endl;
            return;
        }
        int n = warehouses.size();
        vector<bool> visited(n, false);
        CustomQueue<int> q;
        visited[startIdx] = true;
        q.enqueue(startIdx);
        cout << "BFS Traversal starting from " << startWarehouse << ":\n";
        while (!q.isEmpty()) {
            int current = q.peek();
            q.dequeue();
            cout << warehouseIndexMap[current] << " (" << warehouses[current].city << ")";
            for (int neighbor = 0; neighbor < n; ++neighbor) {
                if (adjMatrix[current][neighbor].cost != INF && !visited[neighbor]) {
                    visited[neighbor] = true;
                    q.enqueue(neighbor);
                    cout << " -> " << warehouseIndexMap[neighbor];
                }
            }
            cout << endl;
        }
        cout << endl;
    }
void DFS(const string& startWarehouse) {
    int startIdx = getWarehouseIndex(startWarehouse);
    if (startIdx == -1) {
        cout << "Warehouse not found: " << startWarehouse << endl;
        return;
    }

    int n = warehouses.size();
    vector<bool> visited(n, false);
    CustomStack<int> s;
    cout << "\n===== DFS TRAVERSAL STARTING FROM " << startWarehouse << " =====" << endl;
    cout << "Path: ";

    s.push(startIdx);
    bool firstNode = true;

    while (!s.isEmpty()) {
        int current = s.peek();
        s.pop();

        if (!visited[current]) {
            visited[current] = true;
            
            // Formatting for better output
            if (!firstNode) {
                cout << " -> ";
            } else {
                firstNode = false;
            }
            
            cout << warehouseIndexMap[current] << " (" << warehouses[current].city << ")";
            
            // Push neighbors in reverse order to visit them in correct order
            for (int neighbor = n-1; neighbor >= 0; --neighbor) {
                if (adjMatrix[current][neighbor].cost != INF && !visited[neighbor]) {
                    s.push(neighbor);
                }
            }
        }
    }
    
    // Check if any nodes weren't visited (disconnected graph)
    bool disconnected = false;
    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            if (!disconnected) {
                cout << "\n\nDisconnected components found:";
                disconnected = true;
            }
            cout << "\n  " << warehouseIndexMap[i] << " (" << warehouses[i].city << ")";
        }
    }
    
    cout << "\n\nDFS traversal completed!\n" << endl;
}
};

void displayMainMenu() {
    cout << "\n===== WAREHOUSE MANAGEMENT SYSTEM =====\n";
    cout << "1. Load Data\n";
    cout << "2. Schedule Delivery\n";
    cout << "3. Process Deliveries\n";
    cout << "4. Compute Minimum Spanning Tree\n";
    cout << "5. Perform BFS Traversal\n";
    cout << "6. Perform DFS Traversal\n";
    cout << "7. Exit\n";
    cout << "Enter your choice: ";
}

int main() {
    WarehouseManager manager;
    int choice;
    string deliveryId, warehouseId;
    bool dataLoaded = false;

    do {
        displayMainMenu();
        cin >> choice;
        cin.ignore();
        switch(choice) {
            case 1:
                manager.loadWarehouses("warehouses.txt");
                manager.loadConnections("warehouse_connections.txt");
                manager.loadDeliveries("deliveries.txt");
                dataLoaded = true;
                cout << "Data loaded successfully!\n";
                break;
            case 2:
                if (!dataLoaded) {
                    cout << "Please load data first!\n";
                    break;
                }
                cout << "Enter Delivery ID: ";
                getline(cin, deliveryId);
                manager.scheduleDelivery(deliveryId);
                cout << "Delivery scheduled!\n";
                break;
            case 3:
                if (!dataLoaded) {
                    cout << "Please load data first!\n";
                    break;
                }
                manager.processDeliveries();
                break;
            case 4:
                if (!dataLoaded) {
                    cout << "Please load data first!\n";
                    break;
                }
                cout << "Enter Starting Warehouse ID: ";
                getline(cin, warehouseId);
                manager.computeMST(warehouseId);
                break;
            case 5:
                if (!dataLoaded) {
                    cout << "Please load data first!\n";
                    break;
                }
                cout << "Enter Starting Warehouse ID: ";
                getline(cin, warehouseId);
                manager.BFS(warehouseId);
                break;
            case 6:
                if (!dataLoaded) {
                    cout << "Please load data first!\n";
                    break;
                }
                cout << "Enter Starting Warehouse ID: ";
                getline(cin, warehouseId);
                manager.DFS(warehouseId);
                break;
            case 7:
                cout << "Exiting program...\n";
                break;
            default:
                cout << "Invalid choice! Please try again.\n";
        }
    } while (choice != 7);

    return 0;
}