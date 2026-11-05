class TreeNode(var `val`: Int) {
    var left: TreeNode? = null
    var right: TreeNode? = null
}

// ===== 1 DFS =====

// inv: null base, then combine left and right
fun dfs(node: TreeNode?): Int {
    if (node == null) return 0 // FILL: base
    val l = dfs(node.left)
    val r = dfs(node.right)
    return maxOf(l, r) + 1 // FILL: combine
}

// 104 Maximum Depth of Binary Tree | DFS | FILL: null->0; return maxOf(l, r) + 1
// 543 Diameter of Binary Tree | DFS | FILL: null->0; best = maxOf(best, l + r); return maxOf(l, r) + 1; answer is best (edges)
// 110 Balanced Binary Tree | DFS | FILL: null->0; if (l < 0 || r < 0 || abs(l - r) > 1) return -1; return maxOf(l, r) + 1; tree is balanced iff result >= 0
// 124 Binary Tree Maximum Path Sum | DFS | FILL: null->0; val l = maxOf(0, dfs(left)); val r = maxOf(0, dfs(right)); best = maxOf(best, node.`val` + l + r); return node.`val` + maxOf(l, r); init best = Int.MIN_VALUE
// 112 Path Sum | DFS | FILL: Boolean; null->false; if leaf return node.`val` == sum; else dfs(left, sum - node.`val`) || dfs(right, sum - node.`val`)
// 226 Invert Binary Tree | DFS | FILL: returns TreeNode?; recurse both, then node.left = r; node.right = l; return node

// inv: a subtree returns a found target, or the LCA once both sides are non-null
fun lowestCommonAncestor(node: TreeNode?, p: TreeNode?, q: TreeNode?): TreeNode? {
    if (node == null || node == p || node == q) return node
    val l = lowestCommonAncestor(node.left, p, q)
    val r = lowestCommonAncestor(node.right, p, q)
    if (l != null && r != null) return node
    return l ?: r
}

// 236 Lowest Common Ancestor of a Binary Tree | LCA | FILL: none

// ===== 2 BFS =====

// inv: queue holds the next level; read size before pushing children
fun levelOrder(root: TreeNode?): List<List<Int>> {
    if (root == null) return emptyList()
    val q = ArrayDeque<TreeNode>()
    q.add(root)
    val out = mutableListOf<List<Int>>()
    while (q.isNotEmpty()) {
        val size = q.size
        val level = mutableListOf<Int>()
        repeat(size) {
            val n = q.removeFirst()
            level.add(n.`val`) // FILL: record
            n.left?.let { q.add(it) }
            n.right?.let { q.add(it) }
        }
        out.add(level) // FILL: commit
    }
    return out
}

// 102 Binary Tree Level Order Traversal | BFS | FILL: commit the level list
// 103 Binary Tree Zigzag Level Order Traversal | BFS | FILL: reverse odd levels, then commit
// 199 Binary Tree Right Side View | BFS | FILL: commit level.last() only

// ===== 3 BST =====

// inv: every value lies strictly inside (lo, hi)
fun isValidBST(node: TreeNode?, lo: Long = Long.MIN_VALUE, hi: Long = Long.MAX_VALUE): Boolean {
    if (node == null) return true
    val v = node.`val`.toLong()
    if (v <= lo || v >= hi) return false
    return isValidBST(node.left, lo, v) && isValidBST(node.right, v, hi)
}

// 98 Validate Binary Search Tree | BST | FILL: none — Long bounds, exclusive

// inv: inorder; return on the k-th visit so parents do not keep counting
fun kthSmallest(root: TreeNode?, k0: Int): Int {
    var k = k0
    fun walk(n: TreeNode?): Int? {
        if (n == null) return null
        walk(n.left)?.let { return it }
        if (--k == 0) return n.`val`
        return walk(n.right)
    }
    return walk(root)!!
}

// 230 Kth Smallest Element in a BST | BST | FILL: stop at the k-th inorder visit

// inv: first node that is not strictly on one side of both targets
fun lowestCommonAncestorBST(root: TreeNode?, p: TreeNode, q: TreeNode): TreeNode? {
    var n = root
    while (n != null) {
        n = when {
            p.`val` < n.`val` && q.`val` < n.`val` -> n.left
            p.`val` > n.`val` && q.`val` > n.`val` -> n.right
            else -> return n
        }
    }
    return null
}

// 235 Lowest Common Ancestor of a Binary Search Tree | BST | FILL: none — both smaller left, both larger right, else node

// inv: walk left/right until a null child, then attach
fun insertIntoBST(node: TreeNode?, v: Int): TreeNode {
    if (node == null) return TreeNode(v)
    if (v < node.`val`) node.left = insertIntoBST(node.left, v)
    else node.right = insertIntoBST(node.right, v)
    return node
}

// 701 Insert into a Binary Search Tree | BST | FILL: none

// inv: 0/1 child splices; 2 children copies the successor then deletes the successor
fun deleteNode(node: TreeNode?, key: Int): TreeNode? {
    if (node == null) return null
    if (key < node.`val`) node.left = deleteNode(node.left, key)
    else if (key > node.`val`) node.right = deleteNode(node.right, key)
    else {
        if (node.left == null) return node.right
        if (node.right == null) return node.left
        var s = node.right!!
        while (s.left != null) s = s.left!!
        node.`val` = s.`val`
        node.right = deleteNode(node.right, s.`val`)
    }
    return node
}

// 450 Delete Node in a BST | BST | FILL: none

// ===== 4 Backtracking =====

// inv: path is a combination drawn from nums[start, n)
fun subsets(nums: IntArray): List<List<Int>> {
    val out = mutableListOf<List<Int>>()
    val path = mutableListOf<Int>()
    fun dfs(start: Int) {
        out.add(path.toList()) // FILL: when to record
        for (i in start until nums.size) {
            // FILL: prune
            path.add(nums[i])
            dfs(i + 1) // FILL: i reuses nums[i]; i + 1 takes it once
            path.removeAt(path.lastIndex)
        }
    }
    dfs(0)
    return out
}

// inv: path is a permutation; used[i] means nums[i] is already chosen
fun permute(nums: IntArray): List<List<Int>> {
    val out = mutableListOf<List<Int>>()
    val path = mutableListOf<Int>()
    val used = BooleanArray(nums.size)
    fun dfs() {
        if (path.size == nums.size) {
            out.add(path.toList())
            return
        }
        for (i in nums.indices) {
            if (used[i]) continue
            // FILL: dup prune — sort, then skip i > 0 && nums[i] == nums[i - 1] && !used[i - 1]
            used[i] = true
            path.add(nums[i])
            dfs()
            path.removeAt(path.lastIndex)
            used[i] = false
        }
    }
    dfs()
    return out
}

// 78 Subsets | A | FILL: record every path; dfs(i + 1); no prune
// 90 Subsets II | A | FILL: sort; record every path; prune i > start && nums[i] == nums[i - 1]; dfs(i + 1)
// 46 Permutations | B | FILL: no dup prune
// 47 Permutations II | B | FILL: sort; prune i > 0 && nums[i] == nums[i - 1] && !used[i - 1]
// 39 Combination Sum | A | FILL: record when sum == target; prune sum + nums[i] > target (break only if sorted); dfs(i) reuses nums[i]
// 40 Combination Sum II | A | FILL: sort; record when sum == target; prune sum + nums[i] > target or (i > start && nums[i] == nums[i - 1]); dfs(i + 1)
// 77 Combinations | A | FILL: dfs(1) over 1..n; record when path.size == k; dfs(i + 1); for i in start..(n - (k - path.size) + 1)
// 131 Palindrome Partitioning | A | FILL: record when start == n; for end in start until n; prune if s[start..end] is not a palindrome; dfs(end + 1); path stores substrings
// 17 Letter Combinations of a Phone Number | A | FILL: i walks digits; for c in letters[digits[i]]; record when i == digits.length; dfs(i + 1); empty digits -> empty list
// 79 Word Search | B | FILL: used[r][c]; loop is the 4 neighbors, not 0 until n; prune out of bounds, used, or board[r][c] != word[k]; mark, recurse k + 1, unmark; hit when k == word.length
// 22 Generate Parentheses | PAREN | FILL: open < n append '('; close < open append ')'; record at length 2n

// ===== 5 1D DP =====

// inv: b becomes dp[i], a was dp[i - 1]
fun climbStairs(n: Int): Int {
    var a = 1 // FILL: base
    var b = 1 // FILL: base
    for (i in 2..n) {
        val c = a + b // FILL: 198 maxOf(b, a + x), bases 0, 0 | 91 see MAP
        a = b
        b = c
    }
    return b
}

// inv: dp[i] is true when s[0, i) is a concatenation of dictionary words
fun wordBreak(s: String, wordDict: List<String>): Boolean {
    val dict = wordDict.toHashSet()
    val dp = BooleanArray(s.length + 1)
    dp[0] = true
    for (i in 1..s.length) {
        for (j in 0 until i) {
            if (dp[j] && s.substring(j, i) in dict) {
                dp[i] = true
                break
            }
        }
    }
    return dp[s.length]
}

// 70 Climbing Stairs | DP1 | FILL: a = 1; b = 1; for i in 2..n; c = a + b
// 198 House Robber | ROB | FILL: a = 0; b = 0; for x in nums; c = maxOf(b, a + x)
// 91 Decode Ways | DP1 | FILL: if s.isEmpty() || s[0] == '0' return 0; a = 1; b = 1; for i in 1 until n; two = (s[i - 1] - '0') * 10 + (s[i] - '0'); c = (if s[i] != '0' then b else 0) + (if two in 10..26 then a else 0)
// 139 Word Break | WB | FILL: none — boolean reachability over cuts, not a 2-term roll

// ===== 6 House robber circular =====

// inv: same two-term roll as DP1, on a closed index range
fun rob(nums: IntArray, l: Int = 0, r: Int = nums.lastIndex): Int {
    var a = 0
    var b = 0
    for (i in l..r) {
        val c = maxOf(b, a + nums[i])
        a = b
        b = c
    }
    return b
}

fun robCircular(nums: IntArray): Int {
    if (nums.size == 1) return nums[0]
    return maxOf(rob(nums, 0, nums.lastIndex - 1), rob(nums, 1, nums.lastIndex))
}

// 213 House Robber II | ROB-CIRC | FILL: none — max of drop-first and drop-last; length 1 returns nums[0]

// ===== 7 Knapsack =====

// inv: c walks down so dp[c - w] is still the previous item layer
fun knapsack01(weights: IntArray, values: IntArray, cap: Int): Int {
    val dp = IntArray(cap + 1)
    for (i in weights.indices) {
        for (c in cap downTo weights[i]) {
            dp[c] = maxOf(dp[c], dp[c - weights[i]] + values[i]) // FILL: || or +=
        }
    }
    return dp[cap]
}

// inv: c walks up so dp[c - coin] already includes this coin
fun coinChange(coins: IntArray, amount: Int): Int {
    val dp = IntArray(amount + 1) { amount + 1 }
    dp[0] = 0
    for (coin in coins) {
        for (c in coin..amount) {
            dp[c] = minOf(dp[c], dp[c - coin] + 1) // FILL: += for ways
        }
    }
    return if (dp[amount] > amount) -1 else dp[amount]
}

// 416 Partition Equal Subset Sum | 01 | FILL: odd sum -> false; cap = sum / 2; Boolean dp[0] = true; dp[c] = dp[c] || dp[c - x]; return dp[cap]
// 494 Target Sum | 01 | FILL: (sum + target) odd or abs(target) > sum -> 0; cap = (sum + target) / 2; Int dp[0] = 1; dp[c] += dp[c - x]; return dp[cap]
// 322 Coin Change | UNB | FILL: min; init amount + 1; dp[0] = 0; coin outer, c ascending; -1 if still > amount
// 518 Coin Change II | UNB | FILL: ways; dp[0] = 1; coins OUTER, c ascending; dp[c] += dp[c - coin]
// 377 Combination Sum IV | UNB | FILL: ways; dp[0] = 1; amount OUTER, coins inner; dp[c] += dp[c - coin]

// ===== 8 Stock =====

// inv: snapshot h, s, r before writing; a buy on day p cannot sell on day p
fun stock(prices: IntArray): Int {
    var hold = Int.MIN_VALUE / 2
    var sold = 0
    var rest = 0
    for (p in prices) {
        val h = hold
        val s = sold
        val r = rest
        hold = maxOf(h, r - p) // FILL: 121 -p; 122/714 s - p; 309 r - p
        sold = h + p // FILL: 121/122 maxOf(s, h + p); 714 maxOf(s, h + p - fee); 309 h + p
        rest = maxOf(r, s) // FILL: 309 only
    }
    return maxOf(sold, rest)
}

// 121 Best Time to Buy and Sell Stock | STOCK | FILL: hold = maxOf(h, -p); sold = maxOf(s, h + p); return sold
// 122 Best Time to Buy and Sell Stock II | STOCK | FILL: hold = maxOf(h, s - p); sold = maxOf(s, h + p); return sold
// 714 Best Time to Buy and Sell Stock with Transaction Fee | STOCK | FILL: hold = maxOf(h, s - p); sold = maxOf(s, h + p - fee); return sold
// 309 Best Time to Buy and Sell Stock with Cooldown | STOCK | FILL: hold = maxOf(h, r - p); sold = h + p; rest = maxOf(r, s); return maxOf(sold, rest)

// inv: sell[t] / buy[t] after at most t completed buys; t descends so sell[t - 1] is still yesterday
fun stockK(k0: Int, prices: IntArray): Int {
    val n = prices.size
    if (n == 0) return 0
    val k = k0
    if (k >= n / 2) {
        var acc = 0
        for (i in 1 until n) acc += maxOf(0, prices[i] - prices[i - 1])
        return acc
    }
    val buy = IntArray(k + 1) { Int.MIN_VALUE / 2 }
    val sell = IntArray(k + 1)
    for (p in prices) {
        for (t in k downTo 1) {
            sell[t] = maxOf(sell[t], buy[t] + p)
            buy[t] = maxOf(buy[t], sell[t - 1] - p)
        }
    }
    return sell[k]
}

// 123 Best Time to Buy and Sell Stock III | STOCK-K | FILL: k = 2 — does not fit the 3-variable machine
// 188 Best Time to Buy and Sell Stock IV | STOCK-K | FILL: k as given; k >= n / 2 falls back to unlimited (122)

// ===== 9 LIS =====

// inv: tails[i] is the smallest tail of an increasing subsequence of length i + 1
fun lis(nums: IntArray): Int {
    val tails = mutableListOf<Int>()
    for (x in nums) {
        var lo = 0
        var hi = tails.size
        while (lo < hi) {
            val mid = (lo + hi) ushr 1
            if (tails[mid] < x) lo = mid + 1 else hi = mid
        }
        if (lo == tails.size) tails.add(x) else tails[lo] = x
    }
    return tails.size
}

// inv: dp[i] = LIS length ending at i; cnt[i] = how many such subsequences
fun lis2(a: IntArray): Int {
    val dp = IntArray(a.size) { 1 }
    val cnt = IntArray(a.size) { 1 }
    for (i in a.indices) for (j in 0 until i) if (a[j] < a[i]) {
        if (dp[j] + 1 > dp[i]) { dp[i] = dp[j] + 1; cnt[i] = cnt[j] } // FILL: 300 drop cnt
        else if (dp[j] + 1 == dp[i]) cnt[i] += cnt[j]
    }
    val best = dp.maxOrNull() ?: 0
    return (0 until a.size).sumOf { if (dp[it] == best) cnt[it] else 0 } // FILL: 300 return best
}

// 300 Longest Increasing Subsequence | LIS | FILL: patience length; LIS2 drops cnt and returns best
// 673 Number of Longest Increasing Subsequence | LIS2 | FILL: longer -> cnt[i] = cnt[j]; same length -> cnt[i] += cnt[j]; sum cnt where dp[i] == best

// ===== 10 2D DP =====

// inv: dp[i][j] = LCS length of a[0, i) and b[0, j)
fun lcs(a: String, b: String): Int {
    val m = a.length
    val n = b.length
    val dp = Array(m + 1) { IntArray(n + 1) }
    for (i in 1..m) for (j in 1..n) {
        dp[i][j] = if (a[i - 1] == b[j - 1]) dp[i - 1][j - 1] + 1
        else maxOf(dp[i - 1][j], dp[i][j - 1])
    }
    return dp[m][n]
}

// inv: dp[i][j] = min edits turning a[0, i) into b[0, j)
fun edit(a: String, b: String): Int {
    val m = a.length
    val n = b.length
    val dp = Array(m + 1) { IntArray(n + 1) }
    for (i in 0..m) dp[i][0] = i
    for (j in 0..n) dp[0][j] = j
    for (i in 1..m) for (j in 1..n) {
        dp[i][j] = if (a[i - 1] == b[j - 1]) dp[i - 1][j - 1]
        else 1 + minOf(dp[i - 1][j], dp[i][j - 1], dp[i - 1][j - 1])
    }
    return dp[m][n]
}

// 1143 Longest Common Subsequence | LCS | FILL: none — match: diag + 1; else max(up, left)
// 516 Longest Palindromic Subsequence | LCS | FILL: lcs(s, s.reversed())
// 72 Edit Distance | EDIT | FILL: none — match: diag; else 1 + min(up, left, diag)
// 62 Unique Paths | EDIT | FILL: same up/left nest; dp[i][j] = up + left; first row and first column are 1
// 63 Unique Paths II | EDIT | FILL: obstacle -> 0 else up + left; dp[0][0] = 0 if blocked else 1; a blocked edge cell zeros the cells beyond it
// 64 Minimum Path Sum | EDIT | FILL: dp[i][j] = grid[i][j] + min(up, left); first row and first column are prefix sums

// ===== 11 Interval DP =====

// optional Hard — inv: dp[l][r] = max coins bursting the open interval (l, r); k bursts last
fun maxCoins(nums: IntArray): Int {
    val a = IntArray(nums.size + 2)
    a[0] = 1
    a[a.lastIndex] = 1
    for (i in nums.indices) a[i + 1] = nums[i]
    val n = a.size
    val dp = Array(n) { IntArray(n) }
    for (len in 2 until n) for (l in 0 until n - len) {
        val r = l + len
        for (k in l + 1 until r)
            dp[l][r] = maxOf(dp[l][r], a[l] * a[k] * a[r] + dp[l][k] + dp[k][r])
    }
    return dp[0][n - 1]
}

// 312 Burst Balloons | BURST | FILL: none

// ===== 12 Bits =====

// inv: xor cancels every pair
fun singleNumber(nums: IntArray): Int = nums.reduce { a, b -> a xor b }

// inv: x and (x - 1) clears the lowest set bit
fun hammingWeight(n: Int): Int {
    var x = n.toUInt()
    var c = 0
    while (x != 0u) {
        x = x and (x - 1u)
        c++
    }
    return c
}

// inv: xor 0..n together with every value; the missing index is what remains
fun missingNumber(nums: IntArray): Int {
    var x = nums.size
    for (i in nums.indices) x = x xor i xor nums[i]
    return x
}

// inv: peel the low bit of x onto the high side of r, 32 times
fun reverseBits(n: Int): Int {
    var x = n.toUInt()
    var r = 0u
    repeat(32) {
        r = (r shl 1) or (x and 1u)
        x = x shr 1
    }
    return r.toInt()
}

// 136 Single Number | XOR | FILL: none
// 191 Number of 1 Bits | POP | FILL: none
// 268 Missing Number | MISS | FILL: none
// 190 Reverse Bits | REV | FILL: none

// ===== MISFIT =====

// inv: open stays <= n; close never exceeds open
fun generateParenthesis(n: Int): List<String> {
    val out = mutableListOf<String>()
    val path = StringBuilder()
    fun dfs(open: Int, close: Int) {
        if (path.length == 2 * n) {
            out.add(path.toString())
            return
        }
        if (open < n) {
            path.append('(')
            dfs(open + 1, close)
            path.setLength(path.length - 1)
        }
        if (close < open) {
            path.append(')')
            dfs(open, close + 1)
            path.setLength(path.length - 1)
        }
    }
    dfs(0, 0)
    return out
}

// 22 Generate Parentheses | PAREN | FILL: open < n append '('; close < open append ')'; record at length 2n

// inv: cur is the best subarray sum ending here
fun maxSubArray(nums: IntArray): Int {
    var best = nums[0]
    var cur = nums[0]
    for (i in 1 until nums.size) {
        cur = maxOf(nums[i], cur + nums[i])
        best = maxOf(best, cur)
    }
    return best
}

// 53 Maximum Subarray | KADANE | FILL: cur = maxOf(x, cur + x); best = maxOf(best, cur)

// inv: mx and mn are the best and worst products ending here; a negative flips them
fun maxProduct(nums: IntArray): Int {
    var best = nums[0]
    var mx = nums[0]
    var mn = nums[0]
    for (i in 1 until nums.size) {
        val x = nums[i]
        val a = mx
        mx = maxOf(x, a * x, mn * x)
        mn = minOf(x, a * x, mn * x)
        best = maxOf(best, mx)
    }
    return best
}

// 152 Maximum Product Subarray | KADANE | FILL: keep mn; mx/mn = max/min of (x, oldMx * x, mn * x)

// inv: every center (i, i) and (i, i + 1) grows while the ends match
fun longestPalindrome(s: String): String {
    var count = 0
    var lo = 0
    var len = if (s.isEmpty()) 0 else 1
    fun grow(l0: Int, r0: Int) {
        var l = l0
        var r = r0
        while (l >= 0 && r < s.length && s[l] == s[r]) {
            count++
            if (r - l + 1 > len) {
                lo = l
                len = r - l + 1
            }
            l--
            r++
        }
    }
    for (i in s.indices) {
        grow(i, i)
        grow(i, i + 1)
    }
    return s.substring(lo, lo + len) // FILL: 647 return count
}

// 5 Longest Palindromic Substring | PAL | FILL: expand both centers; keep the longest slice — contiguous, so not LCS
// 647 Palindromic Substrings | PAL | FILL: same expand; each successful step counts one; return count

// inv: popcount(i) = popcount(i / 2) + (i odd)
fun countBits(n: Int): IntArray {
    val dp = IntArray(n + 1)
    for (i in 1..n) dp[i] = dp[i shr 1] + (i and 1)
    return dp
}

// 338 Counting Bits | CBITS | FILL: none — dp[i] = dp[i shr 1] + (i and 1)
