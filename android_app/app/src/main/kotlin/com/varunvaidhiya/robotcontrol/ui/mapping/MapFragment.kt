package com.varunvaidhiya.robotcontrol.ui.mapping

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.viewpager2.adapter.FragmentStateAdapter
import com.google.android.material.tabs.TabLayoutMediator
import com.varunvaidhiya.robotcontrol.databinding.FragmentMapBinding
import com.varunvaidhiya.robotcontrol.ui.slam.SlamMapFragment
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MapFragment : Fragment() {

    private var _binding: FragmentMapBinding? = null
    private val binding get() = _binding!!

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View {
        _binding = FragmentMapBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        binding.viewPagerMap.adapter = object : FragmentStateAdapter(this) {
            override fun getItemCount() = 2
            override fun createFragment(position: Int): Fragment = when (position) {
                0 -> SlamMapFragment()
                else -> PointCloudFragment()
            }
        }

        // Disable swiping on the map pages so pinch-to-zoom works on the SLAM view
        binding.viewPagerMap.isUserInputEnabled = false

        TabLayoutMediator(binding.tabLayoutMap, binding.viewPagerMap) { tab, pos ->
            tab.text = when (pos) {
                0 -> "2D SLAM"
                else -> "3D Cloud"
            }
            tab.icon = when (pos) {
                0 -> requireContext().getDrawable(com.varunvaidhiya.robotcontrol.R.drawable.ic_map)
                else -> requireContext().getDrawable(com.varunvaidhiya.robotcontrol.R.drawable.ic_3d_map)
            }
        }.attach()
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}
