import java.util.PriorityQueue

class ListNode(var `val`: Int) { var next: ListNode? = null }

class RandNode(var `val`: Int) {
    var next: RandNode? = null
    var random: RandNode? = null
}

// S01 哈希探测。map 里只有 i 左边的元素，先查再写。
fun twoSum(nums: IntArray, target: Int): IntArray {
    val map = HashMap<Int, Int>()
    for (i in nums.indices) {
        val need = target - nums[i] // FILL: 探针
        if (need in map) return intArrayOf(map[need]!!, i)
        map[nums[i]] = i // FILL: 存下标还是存布尔
    }
    return intArrayOf()
}
// 1 Two Sum | need = target - x；命中返回两个下标
// 217 Contains Duplicate | need = x；命中返回 true
// 219 Contains Duplicate II | need = x；命中且 i - map[x] <= k
// 290 Word Pattern / 205 Isomorphic | 双向映射，一边失配即 false
// 383 Ransom Note | 计数数组，杂志 -- 到负数即 false
// 202 Happy Number | 把平方和当 key，见过即环

// S02 哈希分桶。key 是规范化后的签名。
fun groupAnagrams(strs: Array<String>): List<List<String>> {
    val map = HashMap<String, MutableList<String>>()
    for (s in strs) {
        val key = String(s.toCharArray().apply { sort() }) // FILL: 签名
        map.getOrPut(key) { mutableListOf() }.add(s)
    }
    return map.values.toList()
}
// 49 Group Anagrams | key = 排序后的字符串，或 26 计数拼成的串
// 242 Valid Anagram | 不建桶：长度相同，++ 再 --，出现负数即 false
// 36 Valid Sudoku | 三个集合："r{i}{d}" "c{j}{d}" "b{i/3}{j/3}{d}"

// S03 哈希连续段。只从一段的左端点开数。
fun longestConsecutive(nums: IntArray): Int {
    val set = nums.toHashSet()
    var best = 0
    for (x in set) {
        if (x - 1 in set) continue // FILL: 不是左端点就跳过
        var y = x
        while (y in set) y++
        best = maxOf(best, y - x)
    }
    return best
}
// 128 Longest Consecutive | 如上
// 846 Hand of Straights | 排序后从最小牌开始，连续 groupSize 张各减 1；用 TreeMap 计数

// S04 前缀和 + 哈希。子数组和 = 当前前缀 - 早先前缀。
fun subarraySum(nums: IntArray, k: Int): Int {
    val map = HashMap<Int, Int>()
    map[0] = 1 // FILL: 计数题放 0->1；最长短题放 0->-1
    var pref = 0
    var ans = 0
    for (x in nums) {
        pref += x
        ans += map[pref - k] ?: 0 // FILL: 探针
        map[pref] = (map[pref] ?: 0) + 1
    }
    return ans
}
// 560 Subarray Sum Equals K | 如上
// 974 Subarray Sums Divisible by K | 探针和写入都用 pref.mod(k)；mod 用 kotlin 的正余数
// 523 Continuous Subarray Sum | map 存最早下标，0->-1；key = if (k==0) pref else pref.mod(k)；i - 下标 >= 2
// 525 Contiguous Array | 0 记成 -1；map 存最早下标；ans = max(i - map[pref])
// 437 Path Sum III | 同一套，前缀改成根到当前结点的和，递归里做完孩子要 map[pref]--
// 930 Binary Subarrays With Sum | 同 560，k = goal
// 238 不是这套，见 C04

// S05 对向双指针。只往里收，丢掉的那一侧不可能更优。
fun twoSumSorted(nums: IntArray, target: Int): IntArray {
    var lo = 0
    var hi = nums.lastIndex
    while (lo < hi) {
        val sum = nums[lo] + nums[hi] // FILL
        if (sum == target) return intArrayOf(lo + 1, hi + 1)
        if (sum < target) lo++ else hi--
    }
    return intArrayOf()
}

fun threeSum(nums: IntArray): List<List<Int>> {
    nums.sort()
    val out = mutableListOf<List<Int>>()
    for (i in nums.indices) {
        if (i > 0 && nums[i] == nums[i - 1]) continue
        var lo = i + 1
        var hi = nums.lastIndex
        val target = -nums[i]
        while (lo < hi) {
            val sum = nums[lo] + nums[hi]
            when {
                sum == target -> {
                    out.add(listOf(nums[i], nums[lo], nums[hi]))
                    lo++; hi--
                    while (lo < hi && nums[lo] == nums[lo - 1]) lo++
                    while (lo < hi && nums[hi] == nums[hi + 1]) hi--
                }
                sum < target -> lo++
                else -> hi--
            }
        }
    }
    return out
}

fun maxArea(height: IntArray): Int {
    var lo = 0
    var hi = height.lastIndex
    var best = 0
    while (lo < hi) {
        best = maxOf(best, (hi - lo) * minOf(height[lo], height[hi]))
        if (height[lo] <= height[hi]) lo++ else hi--
    }
    return best
}
// 167 Two Sum II | twoSumSorted
// 15 3Sum | threeSum，命中后两边都要跳过重复
// 11 Container | maxArea，矮的那边走
// 125 Valid Palindrome | 跳过非字母数字，比较小写
// 5 / 647 不是这套，是中心扩展，在 TreeDpSkeletons.kt
// 42 不是这套，见 C03

// S06 同向写指针。[0, slow) 是保留下来的前缀。
fun removeDuplicates(nums: IntArray): Int {
    var slow = 0
    for (fast in nums.indices) {
        if (slow == 0 || nums[fast] != nums[slow - 1]) { // FILL: 保留条件
            nums[slow] = nums[fast]
            slow++
        }
    }
    return slow
}
// 26 | slow == 0 || nums[fast] != nums[slow - 1]
// 80 | slow < 2 || nums[fast] != nums[slow - 2]
// 27 Remove Element | 保留 nums[fast] != val
// 283 Move Zeroes | 保留 != 0，最后把 [slow, n) 填 0
// 88 Merge Sorted Array | 从尾巴往前写，不是这个循环：i=m-1,j=n-1,k=m+n-1，谁大写谁
// 844 Backspace Compare | 从右往左，跳过 '#' 计数

// S07 荷兰旗。0 往左堆，2 往右堆，1 留在 mid。
fun sortColors(nums: IntArray) {
    var lo = 0
    var mid = 0
    var hi = nums.lastIndex
    while (mid <= hi) {
        when (nums[mid]) {
            0 -> { val t = nums[lo]; nums[lo] = nums[mid]; nums[mid] = t; lo++; mid++ }
            1 -> mid++
            else -> { val t = nums[mid]; nums[mid] = nums[hi]; nums[hi] = t; hi-- }
        }
    }
}
// 75 Sort Colors | 换 2 的时候 mid 不动，因为换进来的值还没看

// S08 滑动窗口，非法才收缩，收缩完再记答案。
fun lengthOfLongestSubstring(s: String): Int {
    val cnt = IntArray(128)
    var lo = 0
    var ans = 0
    for (hi in s.indices) {
        val c = s[hi].code
        cnt[c]++
        while (cnt[c] > 1) { // FILL: 非法条件
            cnt[s[lo].code]--
            lo++
        }
        ans = maxOf(ans, hi - lo + 1)
    }
    return ans
}
// 3 | 当前字符计数 > 1
// 904 Fruit Into Baskets | map.size > 2
// 424 Longest Repeating Character Replacement | 窗口长 - maxFreq > k。maxFreq 只增不减
// 1004 Max Consecutive Ones III | 窗口里 0 的个数 > k
// 219 也可以用这套，但哈希下标更短

// S09 滑动窗口，合法才收缩，收缩前记答案。和 S08 的记录点相反。
fun minSubArrayLen(target: Int, nums: IntArray): Int {
    var lo = 0
    var sum = 0
    var ans = Int.MAX_VALUE
    for (hi in nums.indices) {
        sum += nums[hi]
        while (lo <= hi && sum >= target) { // FILL: 仍然合法
            ans = minOf(ans, hi - lo + 1)
            sum -= nums[lo]
            lo++
        }
    }
    return if (ans == Int.MAX_VALUE) 0 else ans
}

fun minWindow(s: String, t: String): String {
    val need = IntArray(128)
    for (c in t) need[c.code]++
    var missing = t.length
    var lo = 0
    var bestL = 0
    var best = Int.MAX_VALUE
    for (hi in s.indices) {
        if (need[s[hi].code]-- > 0) missing--
        while (missing == 0) {
            if (hi - lo + 1 < best) {
                best = hi - lo + 1
                bestL = lo
            }
            if (need[s[lo].code]++ == 0) missing++
            lo++
        }
    }
    return if (best == Int.MAX_VALUE) "" else s.substring(bestL, bestL + best)
}
// 209 | 和非负，和 >= target 时缩小
// 76 Minimum Window | missing 降到 0 后缩小；need[c]-- > 0 才算补上一个缺口

// S10 定长窗口。hi >= k-1 之后窗口就是 s[hi-k+1, hi]。
fun findAnagrams(s: String, p: String): List<Int> {
    if (p.length > s.length) return emptyList()
    val cnt = IntArray(26)
    for (c in p) cnt[c - 'a']++
    var miss = p.length
    val out = mutableListOf<Int>()
    for (i in s.indices) {
        if (cnt[s[i] - 'a']-- > 0) miss--
        if (i >= p.length && cnt[s[i - p.length] - 'a']++ >= 0) miss++
        if (i >= p.length - 1 && miss == 0) out.add(i - p.length + 1) // FILL: 438 记下标，567 直接 true
    }
    return out
}
// 438 Find All Anagrams | miss == 0 时记录 i - k + 1
// 567 Permutation in String | 同一套，miss == 0 返回 true
// 239 不是定长计数窗口，见 S11

// S11 单调队列。队头是窗口最大值的下标，队内值递减。
fun maxSlidingWindow(nums: IntArray, k: Int): IntArray {
    val dq = ArrayDeque<Int>()
    val ans = IntArray(nums.size - k + 1)
    var w = 0
    for (i in nums.indices) {
        while (dq.isNotEmpty() && dq.first() <= i - k) dq.removeFirst()
        while (dq.isNotEmpty() && nums[dq.last()] <= nums[i]) dq.removeLast()
        dq.addLast(i)
        if (i >= k - 1) ans[w++] = nums[dq.first()]
    }
    return ans
}
// 239 Sliding Window Maximum | 整段就是答案，没有 FILL

// S12 二分下界。lo 含，hi 不含，lo 停在第一个 >= target 的位置。
fun lowerBound(nums: IntArray, target: Int): Int {
    var lo = 0
    var hi = nums.size
    while (lo < hi) {
        val mid = (lo + hi) ushr 1
        if (nums[mid] < target) lo = mid + 1 else hi = mid // FILL: < 是下界，<= 是上界的下一个
    }
    return lo
}
// 35 Search Insert | 直接返回 lo
// 704 Binary Search | lo 落在 target 上才返回，否则 -1
// 34 First and Last | 左 = lower(<)，右 = lower(<=) - 1
// 74 Search a 2D Matrix | 把 m*n 当一维，值是 matrix[mid / n][mid % n]
// 162 Find Peak | 若 nums[mid] < nums[mid+1] 则 lo = mid+1，否则 hi = mid
// 4 Median of Two Sorted Arrays | 不是这套。在短数组上切一刀，使左半边个数对、且左边最大 <= 右边最小

// S13 旋转数组二分。先判断哪一半是有序的。
fun searchRotated(nums: IntArray, target: Int): Int {
    var lo = 0
    var hi = nums.lastIndex
    while (lo <= hi) {
        val mid = (lo + hi) ushr 1
        if (nums[mid] == target) return mid
        if (nums[lo] <= nums[mid]) {
            if (target >= nums[lo] && target < nums[mid]) hi = mid - 1 else lo = mid + 1
        } else {
            if (target > nums[mid] && target <= nums[hi]) lo = mid + 1 else hi = mid - 1
        }
    }
    return -1
}

fun findMinRotated(nums: IntArray): Int {
    var lo = 0
    var hi = nums.lastIndex
    while (lo < hi) {
        val mid = (lo + hi) ushr 1
        if (nums[mid] > nums[hi]) lo = mid + 1 else hi = mid
    }
    return nums[lo]
}
// 33 Search in Rotated | searchRotated。有重复（81）时 nums[lo]==nums[mid] 就 lo++
// 153 Find Min | findMinRotated。有重复（154）时 nums[mid]==nums[hi] 就 hi--

// S14 答案上二分。可行区间是 [答案, +inf)，lo 收到第一个 true。
fun minEatingSpeed(piles: IntArray, h: Int): Int {
    var lo = 1 // FILL: 左端
    var hi = piles.max() // FILL: 右端
    while (lo < hi) {
        val mid = (lo + hi) ushr 1
        var hours = 0L
        for (p in piles) hours += (p + mid - 1L) / mid // FILL: 可行判定
        if (hours <= h) hi = mid else lo = mid + 1
    }
    return lo
}
// 875 Koko | lo=1, hi=max，判定是向上取整的小时和 <= h
// 1011 Capacity / 410 Split Array | lo=max, hi=sum；判定是「按 mid 当容量能切成 <= days 段」
// 69 Sqrt(x) | 找最小的 mid 使 mid.toLong() * mid > x，答案是 mid - 1。方向和 S14 一致
// 778 Swim in Rising Water | 答案是水位 t，判定是网格 BFS 能否只走高度 <= t 的格子

// S15 链表反转。206 是整段反转；92/24/25 是头插窗口。
fun reverseList(head: ListNode?): ListNode? {
    var prev: ListNode? = null
    var cur = head
    while (cur != null) {
        val nxt = cur.next
        cur.next = prev
        prev = cur
        cur = nxt
    }
    return prev
}

fun reverseWindow(head: ListNode?, left: Int, k: Int, once: Boolean): ListNode? {
    val dummy = ListNode(0).apply { next = head }
    var prev = dummy
    repeat(left - 1) { prev = prev.next!! }
    while (true) {
        var probe: ListNode? = prev
        for (t in 0 until k) {
            probe = probe?.next
            if (probe == null) return dummy.next
        }
        val start = prev.next!!
        repeat(k - 1) {
            val node = start.next!!
            start.next = node.next
            node.next = prev.next
            prev.next = node
        }
        if (once) break
        prev = start
    }
    return dummy.next
}
// 206 | reverseList
// 92 Reverse Between | reverseWindow(head, left, right - left + 1, once = true)
// 24 Swap Pairs | reverseWindow(head, 1, 2, once = false)
// 25 Reverse K Group | reverseWindow(head, 1, k, once = false)，不足 k 的尾巴原样留下

// S16 快慢指针。fast 走两步。相遇在环内；不相遇时 slow 停在中点。
fun detectCycle(head: ListNode?): ListNode? {
    var slow = head
    var fast = head
    while (fast?.next != null) {
        slow = slow!!.next
        fast = fast.next!!.next
        if (slow == fast) {
            var p = head
            while (p != slow) {
                p = p!!.next
                slow = slow!!.next
            }
            return p
        }
    }
    return null
}

fun middleNode(head: ListNode?): ListNode? {
    var slow = head
    var fast = head
    while (fast?.next != null) {
        slow = slow!!.next
        fast = fast.next!!.next
    }
    return slow
}

fun isPalindromeList(head: ListNode?): Boolean {
    var right = middleNode(head)
    var prev: ListNode? = null
    while (right != null) {
        val nxt = right.next
        right.next = prev
        prev = right
        right = nxt
    }
    var a = head
    var b = prev
    while (b != null) {
        if (a!!.`val` != b.`val`) return false
        a = a.next
        b = b.next
    }
    return true
}

fun findDuplicate(nums: IntArray): Int {
    var slow = nums[0]
    var fast = nums[0]
    do {
        slow = nums[slow]
        fast = nums[nums[fast]]
    } while (slow != fast)
    slow = nums[0]
    while (slow != fast) {
        slow = nums[slow]
        fast = nums[fast]
    }
    return slow
}
// 141 | detectCycle(head) != null
// 142 | detectCycle 的返回值就是入口
// 876 | middleNode，偶数长度取后一个中点
// 234 | isPalindromeList：中点起反转右半，再逐个比
// 287 Find the Duplicate Number | findDuplicate，下标当链表
// 143 Reorder List | 中点切开 + 反转右半 + 左右交替接。见 reorderList
// 202 Happy Number 也可以快慢，哈希更不容易写错

fun reorderList(head: ListNode?) {
    if (head?.next == null) return
    var slow = head
    var fast = head.next
    while (fast?.next != null) {
        slow = slow!!.next
        fast = fast.next!!.next
    }
    var prev: ListNode? = null
    var cur = slow!!.next
    slow.next = null
    while (cur != null) {
        val nxt = cur.next
        cur.next = prev
        prev = cur
        cur = nxt
    }
    var a = head
    var b = prev
    while (b != null) {
        val an = a!!.next
        val bn = b.next
        a.next = b
        b.next = an
        a = an
        b = bn
    }
}

// S17 合并两条有序链表。dummy 的 next 是结果，tail 永远是最后一个结点。
fun mergeTwoLists(a: ListNode?, b: ListNode?): ListNode? {
    val dummy = ListNode(0)
    var tail = dummy
    var p = a
    var q = b
    while (p != null && q != null) {
        if (p.`val` <= q.`val`) { tail.next = p; p = p.next }
        else { tail.next = q; q = q.next }
        tail = tail.next!!
    }
    tail.next = p ?: q
    return dummy.next
}

fun sortList(head: ListNode?): ListNode? {
    if (head?.next == null) return head
    var slow = head
    var fast = head.next
    while (fast?.next != null) {
        slow = slow!!.next
        fast = fast.next!!.next
    }
    val mid = slow!!.next
    slow.next = null
    return mergeTwoLists(sortList(head), sortList(mid))
}
// 21 | mergeTwoLists
// 148 Sort List | 快慢找中点，断开，归并。合并就是 S17
// 23 不是两两归并的必背版，见 S22

// S18 间距 n。fast 先走 n 步，然后一起走，slow.next 就是要删的。
fun removeNthFromEnd(head: ListNode?, n: Int): ListNode? {
    val dummy = ListNode(0).apply { next = head }
    var fast: ListNode? = dummy
    var slow: ListNode? = dummy
    repeat(n) { fast = fast!!.next }
    while (fast?.next != null) {
        fast = fast!!.next
        slow = slow!!.next
    }
    slow!!.next = slow.next!!.next
    return dummy.next
}
// 19 Remove Nth From End | 整段
// 61 Rotate List | 先数长度，k %= len，快慢相差 len-k，把后半段接到头

// S19 两条链表换头。走到 null 就换到另一条的头，相遇点即交点。
fun getIntersectionNode(a: ListNode?, b: ListNode?): ListNode? {
    var p = a
    var q = b
    while (p != q) {
        p = if (p == null) b else p.next
        q = if (q == null) a else q.next
    }
    return p
}
// 160 Intersection of Two Linked Lists | 整段

// S20 单调栈。栈里下标对应的值不增，弹出时下一个更大元素就是当前下标。
fun dailyTemperatures(t: IntArray): IntArray {
    val ans = IntArray(t.size) // FILL: 739 缺省 0；496/503 缺省 -1
    val st = ArrayDeque<Int>()
    for (i in t.indices) {
        while (st.isNotEmpty() && t[st.last()] < t[i]) { // FILL: 严格小于
            val j = st.removeLast()
            ans[j] = i - j // FILL: 739 写下标差；496 写值
        }
        st.addLast(i)
    }
    return ans
}
// 739 Daily Temperatures | ans[j] = i - j
// 496 Next Greater I | 对 nums2 跑，ans 存值不是距离，再按 nums1 去查
// 503 Next Greater II | 下标跑到 2n，i%n 取值，只在 i < n 时入栈
// 853 Car Fleet | 按位置从右往左，栈里留到达时间，当前时间 <= 栈顶就并进同一车队
// 84 是同一类栈，但宽度公式不同，单独背 S21

// S21 直方图。哨兵高度 0、下标 -1。弹出的柱子是一段的最矮。
fun largestRectangleArea(h: IntArray): Int {
    val st = ArrayDeque<Int>()
    st.addLast(-1)
    var best = 0
    for (i in 0..h.size) {
        val cur = if (i == h.size) 0 else h[i]
        while (st.last() != -1 && h[st.last()] > cur) {
            val mid = st.removeLast()
            best = maxOf(best, h[mid] * (i - st.last() - 1))
        }
        st.addLast(i)
    }
    return best
}
// 84 Largest Rectangle | 整段。宽度 = i - 左边界 - 1
// 85 Maximal Rectangle | 每一行当成直方图的高，套 S21

// S22 括号栈 / 解码栈 / 表达式栈。三种帧，不要混。
fun isValid(s: String): Boolean {
    val st = ArrayDeque<Char>()
    val pair = mapOf(')' to '(', ']' to '[', '}' to '{')
    for (c in s) {
        if (c !in pair) st.addLast(c)
        else if (st.isEmpty() || st.removeLast() != pair[c]) return false
    }
    return st.isEmpty()
}

fun decodeString(s: String): String {
    val counts = ArrayDeque<Int>()
    val parts = ArrayDeque<StringBuilder>()
    var cur = StringBuilder()
    var num = 0
    for (c in s) {
        when {
            c.isDigit() -> num = num * 10 + (c - '0')
            c == '[' -> { counts.addLast(num); parts.addLast(cur); cur = StringBuilder(); num = 0 }
            c == ']' -> {
                val prev = parts.removeLast()
                repeat(counts.removeLast()) { prev.append(cur) }
                cur = prev
            }
            else -> cur.append(c)
        }
    }
    return cur.toString()
}

fun calculate(s: String): Int {
    val st = ArrayDeque<Int>()
    var num = 0
    var sign = '+'
    for (i in s.indices) {
        val c = s[i]
        if (c.isDigit()) num = num * 10 + (c - '0')
        if ((c != ' ' && !c.isDigit()) || i == s.lastIndex) {
            when (sign) {
                '+' -> st.addLast(num)
                '-' -> st.addLast(-num)
                '*' -> st.addLast(st.removeLast() * num)
                '/' -> st.addLast(st.removeLast() / num)
            }
            sign = c
            num = 0
        }
    }
    return st.sum()
}

fun longestValidParentheses(s: String): Int {
    val st = ArrayDeque<Int>()
    st.addLast(-1)
    var best = 0
    for (i in s.indices) {
        if (s[i] == '(') st.addLast(i)
        else {
            st.removeLast()
            if (st.isEmpty()) st.addLast(i) else best = maxOf(best, i - st.last())
        }
    }
    return best
}
// 20 Valid Parentheses | isValid
// 394 Decode String | 帧是 (次数, 左括号前的串)
// 227 Basic Calculator II | 帧是带符号的项，乘除立刻和栈顶合并。Kotlin 除法向 0 截断
// 224 Basic Calculator | 再加一个符号栈，遇到 '(' 把当前 sign 压栈
// 32 Longest Valid Parentheses | 栈里放下标，底放 -1
// 22 括号生成不是栈，是回溯，在 TreeDpSkeletons.kt
// 155 / 150 见单题卡 C07 C08

// S23 大小为 k 的堆。根是下一个该丢掉的。
fun findKthLargest(nums: IntArray, k: Int): Int {
    val pq = PriorityQueue<Int>() // 小根堆
    for (x in nums) {
        pq.add(x)
        if (pq.size > k) pq.poll()
    }
    return pq.peek()
}

fun topKFrequent(nums: IntArray, k: Int): IntArray {
    val freq = HashMap<Int, Int>()
    for (x in nums) freq[x] = (freq[x] ?: 0) + 1
    val pq = PriorityQueue<Int>(compareBy { freq[it] })
    for (key in freq.keys) {
        pq.add(key)
        if (pq.size > k) pq.poll()
    }
    return IntArray(pq.size) { pq.poll()!! }
}
// 215 Kth Largest | 小根堆，容量 k，返回 peek
// 703 Kth Largest in a Stream | 同一个堆，add 时维持 size<=k
// 347 Top K Frequent | 先计数，堆按频次比较
// 973 K Closest | 大根堆按距离，容量 k，满了就丢掉更远的
// 1046 Last Stone Weight | 大根堆，每次弹出两个，差值 > 0 再放回
// 295 不是大小为 k 的堆，见 C14
// 621 Task Scheduler | 不是堆。(maxF-1)*(n+1) + 有多少个任务等于 maxF，再和 tasks.size 取 max

// S24 合并 K 个有序链表。堆里是每条链表的当前头。
fun mergeKLists(lists: Array<ListNode?>): ListNode? {
    val pq = PriorityQueue<ListNode>(compareBy { it.`val` })
    for (n in lists) if (n != null) pq.add(n)
    val dummy = ListNode(0)
    var tail = dummy
    while (pq.isNotEmpty()) {
        val n = pq.poll() ?: break
        tail.next = n
        tail = n
        if (n.next != null) pq.add(n.next!!)
    }
    return dummy.next
}
// 23 Merge k Sorted Lists | 整段
// 373 Find K Pairs with Smallest Sums | 堆里是 (i, j)，弹出后推进 j+1，i 只在 j==0 时推进，避免重复

// S25 区间。按起点排序，只和结果的最后一段比。
fun mergeIntervals(intervals: Array<IntArray>): Array<IntArray> {
    intervals.sortBy { it[0] }
    val res = ArrayDeque<IntArray>()
    for (iv in intervals) {
        if (res.isEmpty() || res.last()[1] < iv[0]) res.addLast(intArrayOf(iv[0], iv[1]))
        else res.last()[1] = maxOf(res.last()[1], iv[1])
    }
    return res.toTypedArray()
}

fun insertInterval(intervals: Array<IntArray>, extra: IntArray): Array<IntArray> {
    val res = ArrayDeque<IntArray>()
    var cur = intArrayOf(extra[0], extra[1])
    for (iv in intervals) {
        when {
            iv[1] < cur[0] -> res.addLast(intArrayOf(iv[0], iv[1]))
            cur[1] < iv[0] -> { res.addLast(cur); cur = intArrayOf(iv[0], iv[1]) }
            else -> { cur[0] = minOf(cur[0], iv[0]); cur[1] = maxOf(cur[1], iv[1]) }
        }
    }
    res.addLast(cur)
    return res.toTypedArray()
}

fun canAttendMeetings(intervals: Array<IntArray>): Boolean {
    intervals.sortBy { it[0] }
    for (i in 1 until intervals.size) if (intervals[i - 1][1] > intervals[i][0]) return false
    return true
}

fun minMeetingRooms(intervals: Array<IntArray>): Int {
    val ev = ArrayList<Pair<Int, Int>>(intervals.size * 2)
    for (iv in intervals) {
        ev.add(iv[0] to 1)
        ev.add(iv[1] to -1)
    }
    ev.sortWith(compareBy({ it.first }, { it.second }))
    var cur = 0
    var best = 0
    for ((_, d) in ev) {
        cur += d
        best = maxOf(best, cur)
    }
    return best
}
// 56 Merge Intervals | 起点排序，lastEnd < start 才新开一段。端点相接要合并
// 57 Insert Interval | 输入已排序。严格在左边的先输出，严格在右边的先把 cur 收掉
// 252 Meeting Rooms | 起点排序，前一个的 end > 下一个的 start 就冲突。相接不算冲突
// 253 Meeting Rooms II | +1 在起点，-1 在终点。同一时刻先处理 -1
// 435 / 452 不是合并，见 S26

// S26 按结束时间排序。能接着放就放，不能放就新开一组。
fun intervalSchedule(a: Array<IntArray>, erase: Boolean): Int {
    if (a.isEmpty()) return 0
    a.sortBy { it[1] }
    var end = a[0][1]
    var groups = 1
    for (i in 1 until a.size) {
        val split = if (erase) a[i][0] >= end else a[i][0] > end
        if (split) {
            groups++
            end = a[i][1]
        }
    }
    return if (erase) a.size - groups else groups
}
// 435 Non-overlapping | erase=true，start >= end 才算能留下，返回 n - 留下的段数
// 452 Minimum Arrows | erase=false，start > end 才需要新箭（端点相接一箭能穿），返回箭数

// S27 最远到达。三道题三份循环，不要合成一个。
fun canJump(nums: IntArray): Boolean {
    var reach = 0
    for (i in nums.indices) {
        if (i > reach) return false
        reach = maxOf(reach, i + nums[i])
    }
    return true
}

fun jump(nums: IntArray): Int {
    var jumps = 0
    var end = 0
    var far = 0
    for (i in 0 until nums.lastIndex) {
        far = maxOf(far, i + nums[i])
        if (i == end) {
            jumps++
            end = far
        }
    }
    return jumps
}

fun canCompleteCircuit(gas: IntArray, cost: IntArray): Int {
    var total = 0
    var tank = 0
    var start = 0
    for (i in gas.indices) {
        val d = gas[i] - cost[i]
        total += d
        tank += d
        if (tank < 0) {
            start = i + 1
            tank = 0
        }
    }
    return if (total >= 0) start else -1
}
// 55 Jump Game | i > reach 就失败
// 45 Jump Game II | 循环停在倒数第二格。i == end 时跳数 +1，end 更新成 far
// 134 Gas Station | 前缀油量变负，起点改到 i+1。总和为负则 -1
// 763 不是这套，见 C12

// S28 Trie。结点是一个前缀，end 表示这里有单词结尾。
class Trie {
    class Node {
        val child = arrayOfNulls<Node>(26)
        var end = false
    }
    val root = Node()

    fun insert(word: String) {
        var n = root
        for (c in word) {
            val i = c - 'a'
            if (n.child[i] == null) n.child[i] = Node()
            n = n.child[i]!!
        }
        n.end = true
    }

    fun search(word: String): Boolean = walk(word)?.end == true
    fun startsWith(prefix: String): Boolean = walk(prefix) != null

    fun searchDot(word: String): Boolean {
        fun dfs(n: Node, i: Int): Boolean {
            if (i == word.length) return n.end
            val c = word[i]
            if (c == '.') {
                for (ch in n.child) if (ch != null && dfs(ch, i + 1)) return true
                return false
            }
            val nxt = n.child[c - 'a'] ?: return false
            return dfs(nxt, i + 1)
        }
        return dfs(root, 0)
    }

    private fun walk(s: String): Node? {
        var n = root
        for (c in s) n = n.child[c - 'a'] ?: return null
        return n
    }
}
// 208 Implement Trie | insert / search / startsWith
// 211 Add and Search Word | addWord = insert，search 里 '.' 走 searchDot
// 212 Word Search II | 把词典插进 Trie，再在棋盘上回溯。命中后把 end 清掉，避免重复收录

// S29 并查集。find 压缩路径，union 按秩，返回 false 表示已经同组。
class UF(n: Int) {
    private val p = IntArray(n) { it }
    private val r = IntArray(n)
    fun find(x: Int): Int {
        if (p[x] != x) p[x] = find(p[x])
        return p[x]
    }
    fun union(a: Int, b: Int): Boolean {
        val pa = find(a)
        val pb = find(b)
        if (pa == pb) return false
        if (r[pa] < r[pb]) p[pa] = pb
        else if (r[pb] < r[pa]) p[pb] = pa
        else { p[pb] = pa; r[pa]++ }
        return true
    }
}
// 547 Number of Provinces | parts 从 n 起，union 成功就 --
// 323 Connected Components | 同 547，边列表版
// 684 Redundant Connection | 结点 1..n，第一根 union 失败的边就是答案
// 261 Graph Valid Tree | 边数必须是 n-1，且中途没有 union 失败
// 721 Accounts Merge | 邮箱当结点，同名账户的邮箱 union，再按根分组
// 1584 Min Cost to Connect All Points | 边按距离排序，Kruskal，union 成功就加距离

// S30 Kahn。边是「先修 -> 课程」，队列里是入度 0。
fun findOrder(n: Int, prereq: Array<IntArray>): IntArray {
    val g = Array(n) { mutableListOf<Int>() }
    val indeg = IntArray(n)
    for (e in prereq) {
        g[e[1]].add(e[0])
        indeg[e[0]]++
    }
    val q = ArrayDeque<Int>()
    for (i in 0 until n) if (indeg[i] == 0) q.addLast(i)
    val order = IntArray(n)
    var k = 0
    while (q.isNotEmpty()) {
        val u = q.removeFirst()
        order[k++] = u
        for (v in g[u]) if (--indeg[v] == 0) q.addLast(v)
    }
    return if (k == n) order else IntArray(0)
}
// 207 Course Schedule | 只返回 k == n
// 210 Course Schedule II | 返回 order，k < n 时返回空数组
// 269 Alien Dictionary | 相邻单词找第一条不同的边，建图后套这套。前一个单词是后一个的前缀则非法

// S31 网格 DFS。踩过就改掉，一次淹没是一个岛。
fun numIslands(grid: Array<CharArray>): Int {
    val m = grid.size
    val n = grid[0].size
    fun dfs(r: Int, c: Int) {
        if (r !in 0 until m || c !in 0 until n || grid[r][c] != '1') return
        grid[r][c] = '0'
        dfs(r + 1, c); dfs(r - 1, c); dfs(r, c + 1); dfs(r, c - 1)
    }
    var count = 0
    for (r in 0 until m) for (c in 0 until n) if (grid[r][c] == '1') {
        dfs(r, c); count++
    }
    return count
}

fun solveSurrounded(board: Array<CharArray>) {
    val m = board.size
    val n = board[0].size
    fun dfs(r: Int, c: Int) {
        if (r !in 0 until m || c !in 0 until n || board[r][c] != 'O') return
        board[r][c] = 'T'
        dfs(r + 1, c); dfs(r - 1, c); dfs(r, c + 1); dfs(r, c - 1)
    }
    for (r in 0 until m) { dfs(r, 0); dfs(r, n - 1) }
    for (c in 0 until n) { dfs(0, c); dfs(m - 1, c) }
    for (r in 0 until m) for (c in 0 until n) {
        if (board[r][c] == 'O') board[r][c] = 'X'
        else if (board[r][c] == 'T') board[r][c] = 'O'
    }
}
// 200 Number of Islands | '1' 改 '0'
// 695 Max Area of Island | 同一次 dfs 里计数，取最大
// 130 Surrounded Regions | 只从边界淹没 'O'，标成 'T'，剩下的 'O' 改 'X'
// 733 Flood Fill | 记下原色，染成新色，原色等于新色直接返回
// 417 Pacific Atlantic | 从四条边往高处走，左右两边都能到的格子留下
// 79 Word Search | 这是回溯不是淹没：要恢复现场。在 TreeDpSkeletons.kt 的 B 骨架

// S32 多源 BFS。第一次入队时距离就定了。
fun orangesRotting(grid: Array<IntArray>): Int {
    val m = grid.size
    val n = grid[0].size
    val q = ArrayDeque<IntArray>()
    var fresh = 0
    for (r in 0 until m) for (c in 0 until n) {
        if (grid[r][c] == 2) q.addLast(intArrayOf(r, c))
        else if (grid[r][c] == 1) fresh++
    }
    if (fresh == 0) return 0
    var minutes = -1
    val dirs = arrayOf(1 to 0, -1 to 0, 0 to 1, 0 to -1)
    while (q.isNotEmpty()) {
        repeat(q.size) {
            val cell = q.removeFirst()
            for ((dr, dc) in dirs) {
                val nr = cell[0] + dr
                val nc = cell[1] + dc
                if (nr !in 0 until m || nc !in 0 until n || grid[nr][nc] != 1) continue
                grid[nr][nc] = 2
                fresh--
                q.addLast(intArrayOf(nr, nc))
            }
        }
        minutes++
    }
    return if (fresh == 0) minutes else -1
}
// 994 Rotting Oranges | 源是 2，墙是 0，答案是层数。没有新鲜橘子返回 0
// 542 01 Matrix | 源是所有 0，空格都能走，距离写进答案数组，不要用格子值当访问标记
// 286 Walls and Gates | 源是门 0，墙是 -1，空房间 INF 被最近的门覆盖
// 127 / 433 是图上的 BFS，见 S35，不是网格

// S33 Dijkstra。边权非负，弹出时 d == dist[u] 才算确定。
fun networkDelayTime(times: Array<IntArray>, n: Int, src: Int): Int {
    val g = Array(n) { mutableListOf<Pair<Int, Int>>() }
    for (e in times) g[e[0] - 1].add(e[1] - 1 to e[2])
    val dist = IntArray(n) { Int.MAX_VALUE }
    dist[src - 1] = 0
    val pq = PriorityQueue<Pair<Int, Int>>(compareBy { it.first })
    pq.add(0 to src - 1)
    while (pq.isNotEmpty()) {
        val (d, u) = pq.poll() ?: break
        if (d != dist[u]) continue
        for ((v, w) in g[u]) {
            val nd = d + w
            if (nd < dist[v]) {
                dist[v] = nd
                pq.add(nd to v)
            }
        }
    }
    var best = 0
    for (d in dist) {
        if (d == Int.MAX_VALUE) return -1
        best = maxOf(best, d)
    }
    return best
}
// 743 Network Delay Time | 结点从 1 编号，答案是 dist 的最大值
// 1584 也可以 Prim，背并查集版即可
// 787 不是 Dijkstra，见 S36

// S34 单词接龙。第一次出队就是最短。这是 BFS，不是 Dijkstra。
fun ladderLength(begin: String, end: String, words: List<String>): Int {
    val dict = words.toMutableSet()
    if (end !in dict) return 0
    dict.remove(begin)
    val q = ArrayDeque<String>()
    q.addLast(begin)
    var steps = 1
    while (q.isNotEmpty()) {
        repeat(q.size) {
            val w = q.removeFirst()
            if (w == end) return steps
            for (i in w.indices) for (c in 'a'..'z') {
                if (c == w[i]) continue
                val nxt = w.substring(0, i) + c + w.substring(i + 1)
                if (nxt in dict) {
                    dict.remove(nxt)
                    q.addLast(nxt)
                }
            }
        }
        steps++
    }
    return 0
}
// 127 Word Ladder | steps 从 1 开始，end 不在词典返回 0
// 433 Minimum Genetic Mutation | steps 从 0 开始，字母表只有 ACGT，失败返回 -1。begin==end 先返回 0
// 909 Snakes and Ladders | 格子 BFS，先把编号映射成坐标，蛇和梯子是瞬移不是边权

// S35 恰好 K 条边的最便宜价格。不要在第一次弹出时定死结点。读 dist，写 next。
fun findCheapestPrice(n: Int, flights: Array<IntArray>, src: Int, dst: Int, k: Int): Int {
    val inf = 1_000_000_000
    var dist = IntArray(n) { inf }
    dist[src] = 0
    repeat(k + 1) {
        val next = dist.copyOf()
        for (e in flights) {
            if (dist[e[0]] == inf) continue
            val nd = dist[e[0]] + e[2]
            if (nd < next[e[1]]) next[e[1]] = nd
        }
        dist = next
    }
    return if (dist[dst] >= inf) -1 else dist[dst]
}
// 787 Cheapest Flights Within K Stops | k 次中转 = k+1 轮松弛

// S36 LRU。head.next 是最新，tail.prev 是最旧。
class LRUCache(private val cap: Int) {
    private class N(val k: Int, var v: Int) {
        var prev: N? = null
        var next: N? = null
    }
    private val map = HashMap<Int, N>()
    private val head = N(0, 0)
    private val tail = N(0, 0)
    init { head.next = tail; tail.prev = head }

    private fun unlink(n: N) {
        n.prev!!.next = n.next
        n.next!!.prev = n.prev
    }
    private fun insertFront(n: N) {
        n.next = head.next
        n.prev = head
        head.next!!.prev = n
        head.next = n
    }
    fun get(key: Int): Int {
        val n = map[key] ?: return -1
        unlink(n)
        insertFront(n)
        return n.v
    }
    fun put(key: Int, value: Int) {
        val old = map[key]
        if (old != null) unlink(old)
        val n = N(key, value)
        insertFront(n)
        map[key] = n
        if (map.size > cap) {
            val lru = tail.prev!!
            unlink(lru)
            map.remove(lru.k)
        }
    }
}
// 146 LRU Cache | get 把结点挪到头。put 已存在的 key 先摘掉，这样满了也不会误删自己。淘汰 tail.prev

// ===== 单题卡。Hot 100 / 高频里不能从上面的骨架硬套的，整段背。 =====

// C01 两数相加。dummy + 进位。
fun addTwoNumbers(a: ListNode?, b: ListNode?): ListNode? {
    val dummy = ListNode(0)
    var tail = dummy
    var p = a
    var q = b
    var carry = 0
    while (p != null || q != null || carry != 0) {
        val sum = (p?.`val` ?: 0) + (q?.`val` ?: 0) + carry
        tail.next = ListNode(sum % 10)
        tail = tail.next!!
        carry = sum / 10
        p = p?.next
        q = q?.next
    }
    return dummy.next
}
// 2 Add Two Numbers | 整段。67 Add Binary 是同一段，换成字符和进位

// C02 随机指针拷贝。两遍哈希，第一遍建结点，第二遍接 next/random。
fun copyRandomList(head: RandNode?): RandNode? {
    if (head == null) return null
    val map = HashMap<RandNode, RandNode>()
    var p: RandNode? = head
    while (p != null) {
        map[p] = RandNode(p.`val`)
        p = p.next
    }
    p = head
    while (p != null) {
        map[p]!!.next = map[p.next]
        map[p]!!.random = map[p.random]
        p = p.next
    }
    return map[head]
}
// 138 Copy List with Random Pointer | 整段

// C03 接雨水。哪边矮走哪边，走的时候用那一侧的最高减去当前高度。
fun trap(h: IntArray): Int {
    var l = 0
    var r = h.lastIndex
    var lm = 0
    var rm = 0
    var ans = 0
    while (l < r) {
        if (h[l] <= h[r]) {
            if (h[l] >= lm) lm = h[l] else ans += lm - h[l]
            l++
        } else {
            if (h[r] >= rm) rm = h[r] else ans += rm - h[r]
            r--
        }
    }
    return ans
}
// 42 Trapping Rain Water | 整段。单调栈是另一份代码，面试写这份

// C04 除自身以外的乘积。左前缀写进答案，右后缀再乘回去。
fun productExceptSelf(nums: IntArray): IntArray {
    val ans = IntArray(nums.size)
    var p = 1
    for (i in nums.indices) { ans[i] = p; p *= nums[i] }
    p = 1
    for (i in nums.lastIndex downTo 0) { ans[i] *= p; p *= nums[i] }
    return ans
}
// 238 Product of Array Except Self | 整段。不要用除法

// C05 轮转数组。整体反转，再反转前 k 个，再反转剩下的。
fun rotateArray(nums: IntArray, k0: Int) {
    val n = nums.size
    if (n == 0) return
    val k = k0 % n
    fun rev(l0: Int, r0: Int) {
        var l = l0
        var r = r0
        while (l < r) {
            val t = nums[l]; nums[l] = nums[r]; nums[r] = t
            l++; r--
        }
    }
    rev(0, n - 1)
    rev(0, k - 1)
    rev(k, n - 1)
}
// 189 Rotate Array | 整段。k %= n

// C06 下一个排列。找到最长不升后缀，和后缀里刚好比它大的交换，再把后缀反转。
fun nextPermutation(nums: IntArray) {
    var i = nums.lastIndex - 1
    while (i >= 0 && nums[i] >= nums[i + 1]) i--
    if (i >= 0) {
        var j = nums.lastIndex
        while (nums[j] <= nums[i]) j--
        val t = nums[i]; nums[i] = nums[j]; nums[j] = t
    }
    var l = i + 1
    var r = nums.lastIndex
    while (l < r) {
        val t = nums[l]; nums[l] = nums[r]; nums[r] = t
        l++; r--
    }
}
// 31 Next Permutation | 整段

// C07 最小栈。平行的一条栈存「到这一层为止的最小值」。
class MinStack {
    private val st = ArrayDeque<Int>()
    private val mn = ArrayDeque<Int>()
    fun push(x: Int) {
        st.addLast(x)
        mn.addLast(if (mn.isEmpty()) x else minOf(x, mn.last()))
    }
    fun pop() { st.removeLast(); mn.removeLast() }
    fun top(): Int = st.last()
    fun getMin(): Int = mn.last()
}
// 155 Min Stack | 整段

// C08 逆波兰。先弹出的是右操作数。
fun evalRPN(tokens: Array<String>): Int {
    val st = ArrayDeque<Int>()
    for (t in tokens) {
        when (t) {
            "+", "-", "*", "/" -> {
                val b = st.removeLast()
                val a = st.removeLast()
                st.addLast(when (t) {
                    "+" -> a + b
                    "-" -> a - b
                    "*" -> a * b
                    else -> a / b
                })
            }
            else -> st.addLast(t.toInt())
        }
    }
    return st.last()
}
// 150 Evaluate RPN | 整段

// C09 多数元素。候选和计数，计数归零就换人。
fun majorityElement(nums: IntArray): Int {
    var cand = 0
    var cnt = 0
    for (x in nums) {
        if (cnt == 0) cand = x
        cnt += if (x == cand) 1 else -1
    }
    return cand
}
// 169 Majority Element | 整段。题目保证一定有多数

// C10 缺失的第一个正数。值 v 放到下标 v-1。
fun firstMissingPositive(nums: IntArray): Int {
    val n = nums.size
    for (i in nums.indices) {
        while (nums[i] in 1..n && nums[nums[i] - 1] != nums[i]) {
            val j = nums[i] - 1
            val t = nums[i]; nums[i] = nums[j]; nums[j] = t
        }
    }
    for (i in nums.indices) if (nums[i] != i + 1) return i + 1
    return n + 1
}
// 41 First Missing Positive | 整段

// C11 矩阵置零。第一行、第一列当标记，col0 单独记。
fun setZeroes(m: Array<IntArray>) {
    val rows = m.size
    val cols = m[0].size
    var col0 = false
    for (r in 0 until rows) {
        if (m[r][0] == 0) col0 = true
        for (c in 1 until cols) if (m[r][c] == 0) {
            m[r][0] = 0
            m[0][c] = 0
        }
    }
    for (r in rows - 1 downTo 0) {
        for (c in cols - 1 downTo 1) if (m[r][0] == 0 || m[0][c] == 0) m[r][c] = 0
        if (col0) m[r][0] = 0
    }
}
// 73 Set Matrix Zeroes | 从底部往回写，避免先弄脏第一行的标记

// C12 划分字母区间。每个字母的最后位置。i 走到 end 就切一刀。
fun partitionLabels(s: String): List<Int> {
    val last = IntArray(26)
    for (i in s.indices) last[s[i] - 'a'] = i
    val out = mutableListOf<Int>()
    var start = 0
    var end = 0
    for (i in s.indices) {
        end = maxOf(end, last[s[i] - 'a'])
        if (i == end) {
            out.add(end - start + 1)
            start = i + 1
        }
    }
    return out
}
// 763 Partition Labels | 整段

// C13 螺旋矩阵。四条边收口，收之前检查上下、左右是否还交叉。
fun spiralOrder(matrix: Array<IntArray>): List<Int> {
    val out = mutableListOf<Int>()
    var top = 0
    var bot = matrix.lastIndex
    var left = 0
    var right = matrix[0].lastIndex
    while (top <= bot && left <= right) {
        for (c in left..right) out.add(matrix[top][c])
        top++
        for (r in top..bot) out.add(matrix[r][right])
        right--
        if (top <= bot) {
            for (c in right downTo left) out.add(matrix[bot][c])
            bot--
        }
        if (left <= right) {
            for (r in bot downTo top) out.add(matrix[r][left])
            left++
        }
    }
    return out
}
// 54 Spiral Matrix | 整段
// 59 Spiral Matrix II | 同一个框，改成按计数填数字

// C14 旋转图像。先沿主对角线交换，再每行反转。
fun rotateImage(m: Array<IntArray>) {
    val n = m.size
    for (r in 0 until n) for (c in r + 1 until n) {
        val t = m[r][c]; m[r][c] = m[c][r]; m[c][r] = t
    }
    for (row in m) {
        var l = 0
        var h = n - 1
        while (l < h) {
            val t = row[l]; row[l] = row[h]; row[h] = t
            l++; h--
        }
    }
}
// 48 Rotate Image | 整段。顺时针。逆时针是先副对角线再反转

// C15 双堆中位数。lo 是大根堆，hi 是小根堆，lo 的个数等于 hi 或比它多 1。
class MedianFinder {
    private val lo = PriorityQueue<Int>(compareByDescending { it })
    private val hi = PriorityQueue<Int>()
    fun addNum(n: Int) {
        lo.add(n)
        hi.add(lo.poll())
        if (hi.size > lo.size) lo.add(hi.poll())
    }
    fun findMedian(): Double {
        return if (lo.size > hi.size) lo.peek()!!.toDouble()
        else (lo.peek()!! + hi.peek()!!) / 2.0
    }
}
// 295 Find Median from Data Stream | 整段

// C16 星号括号。lo 是最少还开着几个，hi 是最多还开着几个。
fun checkValidString(s: String): Boolean {
    var lo = 0
    var hi = 0
    for (c in s) {
        when (c) {
            '(' -> { lo++; hi++ }
            ')' -> { lo--; hi-- }
            else -> { lo--; hi++ }
        }
        if (hi < 0) return false
        if (lo < 0) lo = 0
    }
    return lo == 0
}
// 678 Valid Parenthesis String | 整段。不是 S22 的匹配栈
